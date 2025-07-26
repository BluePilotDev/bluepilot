#!/usr/bin/env python3
import time
import random
import threading
import os
import hashlib
import json
import base64
import glob
import shutil
import subprocess
import fcntl
import errno
from datetime import datetime

from cereal import log

NetworkType = log.DeviceState.NetworkType


def ensure_dependency(package_name, import_name=None):
  """Ensure that a Python package is installed."""
  import_name = import_name or package_name
  try:
    __import__(import_name)
    print(f"{package_name} is already installed.")
    return True
  except ImportError:
    print(f"{package_name} is not installed. Attempting to install...")
    try:
      subprocess.check_call(["pip", "install", package_name])
      try:
        __import__(import_name)
        print(f"{package_name} installed successfully.")
        return True
      except ImportError:
        print(f"Failed to import {package_name} after installation.")
        return False
    except Exception as e:
      print(f"Failed to install {package_name}: {e}")
      return False


# Check required dependencies
if not ensure_dependency("boto3"):
  print("Failed to install boto3. Exiting.")
  exit(1)

if not ensure_dependency("zstandard", "zstandard"):
  print("Failed to install zstandard. Exiting.")
  exit(1)

# Now we can safely import these modules
from boto3 import client
from botocore.client import Config
from botocore.exceptions import ClientError
import zstandard as zstd

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import set_core_affinity
from openpilot.common.swaglog import cloudlog
from bluepilot.logger.bp_logger import info, error

# Constants
ROUTES_DIR = "/data/media/0/realdata"
CACHE_DIR = "/data/bluepilot_data/cache"
TEMP_LOGS_DIR = "/data/bluepilot_data/tmp_logs"
LOCK_DIR = "/data/bluepilot_data/locks"
STATS_FILE = "/data/bluepilot_data/route_sync_stats.json"
SYNC_STATUS_FILE = "/data/bluepilot_data/route_sync_status.json"
ROUTE_STATES_FILE = "/data/bluepilot_data/route_states.json"
SYNC_INTERVAL = 300  # Sync interval in seconds when offroad
MAX_ROUTE_AGE_DAYS = 7  # Only sync routes from the last 7 days

# Encoded S3 Configuration
ENCODED_S3_CONFIG = "ewogICJCVUNLRVRfTkFNRSI6ICJibHVlcGlsb3QtczMiLAogICJFTkRQT0lOVCI6ICJodHRwczovL2JwczMuY3JhenlzYW50aWxsaS5jb20iLAogICJSRUdJT04iOiAidXMtZWFzdC0xIiwKICAiQUNDRVNTX0tFWSI6ICJibHVlcGlsb3RfY29tbWEiLAogICJTRUNSRVRfS0VZIjogIlg3azlQcU0ydkw4blQ1cldqWTR6QjFjSDYiCn0="

allow_sleep = bool(os.getenv("UPLOADER_SLEEP", "1"))
force_wifi = os.getenv("FORCEWIFI") is not None
fake_upload = os.getenv("FAKEUPLOAD") is not None


def update_sync_status(status, route_id=None, details=None):
  """Update the sync status file with the current status."""
  try:
    status_data = {"status": status, "timestamp": time.time(), "route_id": route_id, "details": details or {}}
    os.makedirs(os.path.dirname(SYNC_STATUS_FILE), exist_ok=True)
    with open(SYNC_STATUS_FILE, 'w') as f:
      json.dump(status_data, f)
  except Exception as e:
    error(f"Failed to update sync status: {e}", True)


def get_s3_config():
  """Decode the S3 configuration."""
  config_json = base64.b64decode(ENCODED_S3_CONFIG).decode('utf-8')
  return json.loads(config_json)


def init_s3_client():
  """Initialize and test the S3 client."""
  s3_config = get_s3_config()
  endpoint = s3_config.get("ENDPOINT", "")
  region = s3_config.get("REGION", "")
  access_key = s3_config.get("ACCESS_KEY", "")

  try:
    s3_client = client(
      's3',
      endpoint_url=endpoint,
      region_name=region,
      aws_access_key_id=access_key,
      aws_secret_access_key=s3_config["SECRET_KEY"],
      config=Config(signature_version='s3v4', connect_timeout=30, read_timeout=60, retries={'max_attempts': 5}),
    )
    # The above code is a Python script that prints a message indicating a successful S3 connection
    # with the signature version set to "s3v4".
    # response = s3_client.list_buckets()
    # print(f"S3 connection successful with signature_version=s3v4")
    return s3_client, s3_config
  except Exception as e:
    print(f"Failed to initialize S3 client: {e}")
    return None, s3_config


def get_route_segments(routes_dir):
  """Get all route segments from the specified directory."""
  if not os.path.exists(routes_dir):
    print(f"Warning: Routes directory {routes_dir} does not exist!")
    return []

  pattern = os.path.join(routes_dir, "[0-9a-f]*--[0-9a-f]*--[0-9]*")
  segment_paths = glob.glob(pattern)
  return segment_paths


def group_route_segments(segment_paths):
  """Group route segments by their route ID."""
  route_groups = {}
  for segment_path in segment_paths:
    segment_name = os.path.basename(segment_path)
    parts = segment_name.split('--')
    if len(parts) == 3:
      route_id = f"{parts[0]}--{parts[1]}"
      segment_num = int(parts[2])

      if route_id not in route_groups:
        route_groups[route_id] = []

      route_groups[route_id].append((segment_num, segment_path))

  # Sort segments within each route
  for route_id in route_groups:
    route_groups[route_id].sort(key=lambda x: x[0])

  return route_groups


def get_route_timestamp(segment_path):
  """Extract the timestamp from the first segment of a route."""
  try:
    # Try to get timestamp from rlog metadata
    rlog_path = os.path.join(segment_path, "rlog.zst")
    if not os.path.exists(rlog_path):
      rlog_path = os.path.join(segment_path, "rlog")
      if not os.path.exists(rlog_path):
        return None

    # Use the file creation time as a proxy for the route timestamp
    return os.path.getctime(rlog_path)
  except Exception as e:
    error(f"Failed to get route timestamp: {e}", True)
    return None


def format_timestamp(timestamp):
  """Format a timestamp in a human-readable format."""
  if timestamp is None:
    return "Unknown"
  return datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")


def format_file_size(size_bytes):
  """Format file size in bytes to human-readable format."""
  if size_bytes < 1024:
    return f"{size_bytes} B"
  elif size_bytes < 1024 * 1024:
    return f"{size_bytes / 1024:.2f} KB"
  elif size_bytes < 1024 * 1024 * 1024:
    return f"{size_bytes / (1024 * 1024):.2f} MB"
  else:
    return f"{size_bytes / (1024 * 1024 * 1024):.2f} GB"


def is_route_recent(route_timestamp, max_age_days=7):
  """Check if the route is within the specified number of days old."""
  if route_timestamp is None:
    return False
  cutoff_time = time.time() - (max_age_days * 24 * 60 * 60)
  return route_timestamp >= cutoff_time


def verify_s3_upload(s3_client, s3_config, route_key, expected_md5, expected_size):
  try:
    # Get object metadata from S3
    head_response = s3_client.head_object(Bucket=s3_config["BUCKET_NAME"], Key=route_key)

    # Check if file size matches
    if head_response.get('ContentLength') != expected_size:
      print(f"Size mismatch: expected {expected_size}, got {head_response.get('ContentLength')}")
      return False

    # For non-multipart uploads, ETag is the MD5 hash (in quotes)
    s3_etag = head_response.get('ETag', '').strip('"')

    # If this is a simple upload, ETag should match our MD5
    if s3_etag == expected_md5:
      return True

    # For multipart uploads, we'd need more complex verification
    # This is simplified - real implementation would depend on upload method
    print(f"ETag verification: expected {expected_md5}, got {s3_etag}")
    return False

  except Exception as e:
    print(f"Failed to verify upload: {e}")
    return False


def is_route_uploaded(s3_client, s3_config, dongle_id, route_id):
  """Check if a route has already been uploaded to S3."""
  try:
    # Check S3 first by looking for both metadata and rlog files
    prefix = f"data_collection/{dongle_id}/routes/{route_id}"
    response = s3_client.list_objects_v2(Bucket=s3_config["BUCKET_NAME"], Prefix=prefix, MaxKeys=10)

    metadata_found = False
    rlog_found = False

    if 'Contents' in response:
      for item in response['Contents']:
        key = item.get('Key', '')
        if "_metadata.json" in key:
          metadata_found = True
        if "_rlog.zst" in key or "_rlog" in key:
          rlog_found = True

    is_in_s3 = metadata_found and rlog_found

    # Update local stats if they don't match S3
    sync_stats = load_sync_stats()
    route_in_stats = route_id in sync_stats.get("routes", {})
    marked_as_uploaded = route_in_stats and sync_stats["routes"][route_id].get("uploaded", False)

    if marked_as_uploaded != is_in_s3:
      # Local stats don't match S3 reality - update them
      if route_in_stats:
        sync_stats["routes"][route_id]["uploaded"] = is_in_s3
        if not is_in_s3:
          print(f"Route {route_id} marked for re-upload: not found in S3")
      save_sync_stats(sync_stats)

    if is_in_s3:
      # Only log this when actually found in S3
      print(f"Found both metadata and rlog files for route {route_id}")

    return is_in_s3
  except Exception as e:
    error(f"Failed to check if route {route_id} is uploaded: {e}", True)
    return False


def get_route_states():
  """Load route processing states from file."""
  states = {}
  try:
    if os.path.exists(ROUTE_STATES_FILE):
      with open(ROUTE_STATES_FILE) as f:
        states = json.load(f)
  except Exception as e:
    error(f"Failed to load route states: {e}", True)
  return states


def save_route_states(states):
  """Save route processing states to file."""
  try:
    os.makedirs(os.path.dirname(ROUTE_STATES_FILE), exist_ok=True)
    with open(ROUTE_STATES_FILE, 'w') as f:
      json.dump(states, f)
  except Exception as e:
    error(f"Failed to save route states: {e}", True)


def acquire_route_lock(route_id):
  """Attempt to acquire a lock for processing a specific route."""
  os.makedirs(LOCK_DIR, exist_ok=True)
  lock_file = os.path.join(LOCK_DIR, f"{route_id}.lock")

  try:
    # Open the lock file (create if doesn't exist)
    fd = open(lock_file, 'w')

    # Try to acquire an exclusive lock, non-blocking
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)

    # If we get here, we acquired the lock
    return fd
  except OSError as e:
    # Failed to acquire lock
    if e.errno == errno.EAGAIN:
      print(f"Route {route_id} is already being processed by another instance")
    else:
      error(f"Error acquiring lock for route {route_id}: {e}", True)
    return None


def release_route_lock(fd, route_id):
  """Release a previously acquired route lock."""
  if fd:
    try:
      fcntl.flock(fd, fcntl.LOCK_UN)
      fd.close()
      # Optionally remove the lock file
      lock_file = os.path.join(LOCK_DIR, f"{route_id}.lock")
      if os.path.exists(lock_file):
        os.remove(lock_file)
    except Exception as e:
      error(f"Error releasing lock for route {route_id}: {e}", True)


def process_route(route_id, segments, cache_dir, dongle_id):
  """Process a route by extracting relevant data."""
  try:
    update_sync_status("processing", route_id, {"stage": "processing_route", "segments": len(segments)})
    os.makedirs(cache_dir, exist_ok=True)

    # Check if route is already processed and cached
    cache_file = os.path.join(cache_dir, f"{route_id}.json")
    if os.path.exists(cache_file):
      with open(cache_file) as f:
        return json.load(f)

    # Extract timestamp from the first segment
    first_segment_path = segments[0][1]
    route_timestamp = get_route_timestamp(first_segment_path)

    if route_timestamp is None:
      error(f"Could not determine timestamp for route {route_id}", True)
      update_sync_status("error", route_id, {"message": "Could not determine timestamp"})
      return None

    # Process route data
    route_data = {
      "route_id": route_id,
      "timestamp": route_timestamp,
      "dongle_id": dongle_id,
      "segment_count": len(segments),
      "processed_at": time.time(),
      "segments": [seg_path for _, seg_path in segments],
      "human_timestamp": format_timestamp(route_timestamp),
    }

    # Cache the processed data
    with open(cache_file, 'w') as f:
      json.dump(route_data, f)

    return route_data

  except Exception as e:
    error(f"Failed to process route {route_id}: {e}", True)
    update_sync_status("error", route_id, {"message": f"Failed to process route: {e}"})
    return None


def concatenate_route_segments(segments, temp_dir, route_id):
  try:
    update_sync_status("processing", route_id, {"stage": "concatenating_segments", "segments": len(segments)})
    os.makedirs(temp_dir, exist_ok=True)
    concat_file_path = os.path.join(temp_dir, "concatenated_rlog")
    final_file_path = os.path.join(temp_dir, "concatenated_rlog.zst")

    # Step 1: Concatenate all segments into an uncompressed file
    with open(concat_file_path, 'wb') as outfile:
      segment_count = 0
      for _, segment_path in segments:
        segment_count += 1
        update_sync_status("processing", route_id, {"stage": "concatenating_segments", "current": segment_count, "total": len(segments)})

        rlog_zst_path = os.path.join(segment_path, "rlog.zst")
        rlog_path = os.path.join(segment_path, "rlog")

        if os.path.exists(rlog_zst_path):
          with open(rlog_zst_path, 'rb') as infile:
            dctx = zstd.ZstdDecompressor()
            reader = dctx.stream_reader(infile)
            buffer_size = 8192
            while True:
              chunk = reader.read(buffer_size)
              if not chunk:
                break
              outfile.write(chunk)
        elif os.path.exists(rlog_path):
          with open(rlog_path, 'rb') as infile:
            shutil.copyfileobj(infile, outfile, 8192)
        else:
          raise FileNotFoundError(f"No rlog or rlog.zst found in {segment_path}")

    # Step 2: Compress the concatenated file into .zst
    with open(concat_file_path, 'rb') as infile:
      with open(final_file_path, 'wb') as outfile:
        cctx = zstd.ZstdCompressor(level=3)  # Match LOG_COMPRESSION_LEVEL if known
        writer = cctx.stream_writer(outfile)
        shutil.copyfileobj(infile, writer, 8192)
        writer.flush(zstd.FLUSH_FRAME)  # Ensure complete Zstandard frame

    # Clean up temporary uncompressed file
    os.remove(concat_file_path)
    return final_file_path
  except Exception as e:
    error(f"Failed to concatenate route segments: {e}", True)
    update_sync_status("error", route_id, {"message": f"Failed to concatenate segments: {e}"})
    return None


def upload_route(s3_client, s3_config, route_data, dongle_id, temp_dir):
  """Upload route data to S3 with integrity verification."""
  try:
    route_id = route_data["route_id"]
    route_timestamp = route_data["timestamp"]
    timestamp_str = datetime.fromtimestamp(route_timestamp).strftime("%Y%m%d_%H%M%S")
    print(f"Starting upload for route {route_id} at {timestamp_str}")

    # Create temporary directory for processing
    temp_concat_dir = os.path.join(TEMP_LOGS_DIR, route_id)
    os.makedirs(TEMP_LOGS_DIR, exist_ok=True)

    # Concatenate route segments
    concat_file_path = concatenate_route_segments([(i, path) for i, path in enumerate(route_data["segments"])], temp_concat_dir, route_id)

    if concat_file_path is None:
      print("Failed to concatenate segments")
      return False

    # Calculate MD5 hash of the concatenated file
    md5_hash = hashlib.md5()
    with open(concat_file_path, 'rb') as f:
      for chunk in iter(lambda: f.read(4096), b''):
        md5_hash.update(chunk)
    md5_digest = md5_hash.hexdigest()
    file_size = os.path.getsize(concat_file_path)

    # Create metadata with MD5 and file size information
    metadata = {
      "route_id": route_id,
      "dongle_id": dongle_id,
      "timestamp": route_timestamp,
      "segment_count": route_data["segment_count"],
      "processed_at": time.time(),
      "upload_timestamp": time.time(),
      "human_timestamp": format_timestamp(route_timestamp),
      "rlog_md5": md5_digest,
      "rlog_size": file_size,
    }

    metadata_file_path = os.path.join(temp_concat_dir, "metadata.json")
    with open(metadata_file_path, 'w') as f:
      json.dump(metadata, f)

    # Define keys
    metadata_key = f"data_collection/{dongle_id}/routes/{route_id}_metadata.json"
    route_key = f"data_collection/{dongle_id}/routes/{route_id}_rlog.zst"

    # Upload route data with transfer rate monitoring
    print(f"Uploading route data ({format_file_size(file_size)}) to {route_key}")

    class ProgressPercentage:
      def __init__(self, filename):
        self._filename = filename
        self._size = float(os.path.getsize(filename))
        self._seen_so_far = 0
        self._start_time = time.time()
        self._last_update_time = time.time()
        self._lock = threading.Lock()
        self._update_interval = 2.0  # Update every 2 seconds

      def __call__(self, bytes_amount):
        with self._lock:
          self._seen_so_far += bytes_amount
          current_time = time.time()
          if current_time - self._last_update_time > self._update_interval:
            # Calculate transfer rate
            elapsed = current_time - self._start_time
            if elapsed > 0:
              transfer_rate = self._seen_so_far / elapsed
              percentage = (self._seen_so_far / self._size) * 100
              print(
                f"Upload progress: {format_file_size(self._seen_so_far)}/{format_file_size(self._size)} ({percentage:.1f}%) at {format_file_size(transfer_rate)}/s"
              )
            self._last_update_time = current_time

            if self._seen_so_far >= self._size:
              print(f"Upload complete in {elapsed:.1f}s (avg: {format_file_size(self._size / elapsed)}/s)")

    try:
      # Use the progress callback
      progress_callback = ProgressPercentage(concat_file_path)
      s3_client.upload_file(
        Filename=concat_file_path, Bucket=s3_config["BUCKET_NAME"], Key=route_key, ExtraArgs={'ContentType': 'application/zstd'}, Callback=progress_callback
      )
      print(f"Route data upload completed for {route_id}, verifying integrity...")
    except Exception as e:
      print(f"Route data upload failed: {e}")
      return False

    # Verify the upload was successful by checking S3 metadata
    try:
      head_response = s3_client.head_object(Bucket=s3_config["BUCKET_NAME"], Key=route_key)

      # Check file size
      if head_response.get('ContentLength') != file_size:
        print(f"Size mismatch for route {route_id}: expected {file_size}, got {head_response.get('ContentLength')}")

        # Delete the incomplete file from S3
        s3_client.delete_object(Bucket=s3_config["BUCKET_NAME"], Key=route_key)
        print(f"Deleted incomplete upload for route {route_id}")
        return False

      # For non-multipart uploads, ETag is the MD5 hash (in quotes)
      s3_etag = head_response.get('ETag', '').strip('"')

      # Note: For multipart uploads, ETag won't match our MD5
      # This is simplified - in practice, you might need to check if a multipart upload was used
      if s3_etag != md5_digest:
        print(f"MD5 mismatch for route {route_id}: expected {md5_digest}, got {s3_etag}")
        print("Note: This might be expected for multipart uploads")
        # Optionally implement additional verification for multipart uploads

      print(f"Route data verification completed for {route_id}")
    except Exception as e:
      print(f"Failed to verify route data: {e}")
      return False

    # Upload metadata only after successful rlog verification
    print(f"Uploading metadata to {metadata_key}")
    try:
      with open(metadata_file_path, 'rb') as f:
        s3_client.put_object(Bucket=s3_config["BUCKET_NAME"], Key=metadata_key, Body=f.read(), ContentType='application/json')
    except Exception as e:
      print(f"Metadata upload failed: {e}")

      # Delete the rlog file from S3 since metadata upload failed
      try:
        s3_client.delete_object(Bucket=s3_config["BUCKET_NAME"], Key=route_key)
        print(f"Deleted rlog file due to metadata upload failure for route {route_id}")
      except ClientError:
        # File might not exist, that's okay
        pass

      return False

    # Final verification - ensure both files exist
    try:
      response = s3_client.list_objects_v2(Bucket=s3_config["BUCKET_NAME"], Prefix=f"data_collection/{dongle_id}/routes/{route_id}")

      metadata_found = False
      rlog_found = False

      if 'Contents' in response:
        for item in response['Contents']:
          key = item.get('Key', '')
          if metadata_key == key:
            metadata_found = True
          if route_key == key:
            rlog_found = True

      if not (metadata_found and rlog_found):
        print(f"Final verification failed: metadata_found={metadata_found}, rlog_found={rlog_found}")
        return False

    except Exception as e:
      print(f"Final verification failed: {e}")
      return False

    # Cleanup only after everything is verified
    try:
      os.remove(concat_file_path)
      os.remove(metadata_file_path)
      shutil.rmtree(temp_concat_dir, ignore_errors=True)
    except Exception as e:
      print(f"Warning: Failed to clean up temporary files: {e}")

    print(f"Successfully uploaded and verified route {route_id} to S3")
    return True

  except Exception as e:
    print(f"Failed to upload route {route_id}: {e}")
    return False


def load_sync_stats():
  """Load sync stats from file or initialize if file doesn't exist."""
  stats = {
    "total_routes_found": 0,
    "total_routes_uploaded": 0,
    "last_sync_timestamp": 0,
    "last_sync_route_id": "",
    "successful_syncs": 0,
    "failed_syncs": 0,
    "routes": {},
  }

  try:
    if os.path.exists(STATS_FILE):
      with open(STATS_FILE) as f:
        loaded_stats = json.load(f)
        stats.update(loaded_stats)
  except Exception as e:
    error(f"Failed to load sync stats: {e}", True)

  return stats


def refresh_sync_stats_from_s3(s3_client, s3_config, dongle_id):
  """Scan S3 to get an accurate list of uploaded routes."""
  print("Refreshing sync stats from S3...")

  # First clean up any incomplete uploads
  cleanup_count = cleanup_incomplete_uploads(s3_client, s3_config, dongle_id)
  if cleanup_count > 0:
    print(f"Cleaned up {cleanup_count} incomplete uploads")

  # Load current stats
  sync_stats = load_sync_stats()

  try:
    # List all objects in the dongle's routes directory
    prefix = f"data_collection/{dongle_id}/routes/"

    # Use pagination to handle large numbers of routes
    paginator = s3_client.get_paginator('list_objects_v2')

    # Group metadata and route files
    route_files = {}

    for page in paginator.paginate(Bucket=s3_config["BUCKET_NAME"], Prefix=prefix):
      if 'Contents' not in page:
        continue

      for item in page['Contents']:
        key = item['Key']
        # Extract route_id from the key
        filename = os.path.basename(key)
        if "_metadata.json" in filename:
          route_id = filename.split('_metadata.json')[0]
          if route_id not in route_files:
            route_files[route_id] = {'metadata': True}
          else:
            route_files[route_id]['metadata'] = True
        elif "_rlog.zst" in filename or "_rlog" in filename:
          route_id = filename.split('_rlog')[0]
          if route_id not in route_files:
            route_files[route_id] = {'rlog': True}
          else:
            route_files[route_id]['rlog'] = True

    # Consider routes with both metadata and rlog as fully uploaded
    uploaded_routes = set()
    for route_id, files in route_files.items():
      if files.get('metadata', False) and files.get('rlog', False):
        uploaded_routes.add(route_id)

    # IMPORTANT: Mark routes as not uploaded if they're not in S3
    for route_id in list(sync_stats["routes"].keys()):
      if route_id not in uploaded_routes:
        if sync_stats["routes"][route_id].get("uploaded", False):
          print(f"Route {route_id} marked for re-upload: not found in S3")
          sync_stats["routes"][route_id]["uploaded"] = False
      else:
        sync_stats["routes"][route_id]["uploaded"] = True

    # Update the sync_stats with the S3 information
    for route_id in uploaded_routes:
      if route_id not in sync_stats["routes"]:
        # Create a minimal entry if we don't have local info
        sync_stats["routes"][route_id] = {
          "found": True,
          "uploaded": True,
          "timestamp": time.time(),  # Use current time as we don't know the real timestamp
          "upload_timestamp": time.time(),
          "segment_count": 0,  # Unknown
          "fingerprint": "unknown",
        }

    # Recalculate the total uploaded count accurately
    uploaded_count = sum(1 for r in sync_stats["routes"].values() if r.get("uploaded", False))
    sync_stats["total_routes_uploaded"] = uploaded_count

    print(f"S3 scan complete: found {len(uploaded_routes)} uploaded routes")
    save_sync_stats(sync_stats)
    return True

  except Exception as e:
    error(f"Failed to refresh sync stats from S3: {e}", True)
    return False


def save_sync_stats(stats):
  """Save sync stats to file."""
  try:
    os.makedirs(os.path.dirname(STATS_FILE), exist_ok=True)
    with open(STATS_FILE, 'w') as f:
      json.dump(stats, f)
  except Exception as e:
    error(f"Failed to save sync stats: {e}", True)


def get_fingerprint_from_route(route_id, segments):
  """Try to extract vehicle fingerprint from a route."""
  # This is a placeholder - in a real implementation, you would extract
  # the fingerprint from the car state in the rlog file
  return "unknown"


def sync_routes(network_type, metered=False):
  """Sync route data with S3 when network is available."""
  print(f"Syncing routes with network_type={network_type}, metered={metered}")
  update_sync_status("starting", details={"network_type": str(network_type)})

  # Define timeouts for route processing
  PROCESSING_TIMEOUT = 3600  # 1 hour
  FAILED_RETRY_TIMEOUT = 1800  # 30 minutes

  params = Params()
  dongle_id = params.get("DongleId").decode('utf-8') if params.get("DongleId") else "unknown"
  cache_dir = os.path.join(CACHE_DIR, dongle_id)
  temp_dir = os.path.join("/tmp", "bluepilot_route_sync")

  # Initialize S3 client
  s3_client, s3_config = init_s3_client()
  if s3_client is None:
    update_sync_status("error", details={"message": "Failed to initialize S3 client"})
    return 0  # No routes to upload if S3 client fails

  # Refresh stats from S3 periodically (every 24 hours)
  sync_stats = load_sync_stats()
  last_refresh = sync_stats.get("last_s3_refresh", 0)
  current_time = time.time()
  if current_time - last_refresh > 86400:
    print("Stats refresh needed")
    refresh_sync_stats_from_s3(s3_client, s3_config, dongle_id)
    sync_stats = load_sync_stats()
    sync_stats["last_s3_refresh"] = current_time
    save_sync_stats(sync_stats)

  # Load route states
  route_states = get_route_states()

  # Identify and group route segments
  update_sync_status("scanning", details={"stage": "finding_routes"})
  segment_paths = get_route_segments(ROUTES_DIR)
  route_groups = group_route_segments(segment_paths)

  # Filter to recent routes and sort by timestamp (newest first)
  update_sync_status("scanning", details={"stage": "filtering_routes", "found": len(route_groups)})
  recent_routes = []
  for route_id, segments in route_groups.items():
    if segments:
      first_segment_path = segments[0][1]
      route_timestamp = get_route_timestamp(first_segment_path)
      fingerprint = get_fingerprint_from_route(route_id, segments)
      if route_timestamp and is_route_recent(route_timestamp, MAX_ROUTE_AGE_DAYS):
        recent_routes.append((route_id, segments, route_timestamp, fingerprint))

  recent_routes.sort(key=lambda x: x[2], reverse=True)
  print(f"Found {len(recent_routes)} recent routes to process")

  if not recent_routes:
    update_sync_status("idle", details={"message": "No recent routes to sync"})
    return 0

  # Calculate initial routes to upload
  recent_route_ids = set(route_id for route_id, _, _, _ in recent_routes)
  routes_to_upload = sum(1 for route_id in recent_route_ids if not is_route_uploaded(s3_client, s3_config, dongle_id, route_id))

  if routes_to_upload == 0:
    print(f"All recent routes are already uploaded ({len(recent_routes)} routes checked)")
    update_sync_status("idle", details={"message": "No new routes to upload"})
    return 0

  print(f"Found {routes_to_upload} recent routes that need uploading out of {len(recent_routes)} recent routes")

  # Process and upload one route
  for route_id, segments, route_timestamp, fingerprint in recent_routes:
    if is_route_uploaded(s3_client, s3_config, dongle_id, route_id):
      continue

    # Handle route states
    route_state = route_states.get(route_id, {})
    current_time = time.time()
    state_status = route_state.get('status')

    if state_status == 'processing':
      start_time = route_state.get('start_time', 0)
      if current_time - start_time < PROCESSING_TIMEOUT:
        info(f"Route {route_id} is already being processed, skipping", True)
        continue
      else:
        info(f"Route {route_id} processing timed out after {PROCESSING_TIMEOUT}s, will retry", True)

    elif state_status == 'failed':
      failure_time = route_state.get('failure_time', 0)
      if current_time - failure_time < FAILED_RETRY_TIMEOUT:
        info(f"Route {route_id} failed recently, waiting before retry", True)
        continue
      else:
        info(f"Retrying previously failed route {route_id}", True)

    lock_fd = acquire_route_lock(route_id)
    if not lock_fd:
      continue

    try:
      route_states[route_id] = {'status': 'processing', 'start_time': time.time()}
      save_route_states(route_states)

      route_data = process_route(route_id, segments, cache_dir, dongle_id)
      if route_data is None:
        route_states[route_id] = {'status': 'failed', 'failure_time': time.time(), 'error': 'Processing failed'}
        save_route_states(route_states)
        continue

      route_data["fingerprint"] = fingerprint
      success = upload_route(s3_client, s3_config, route_data, dongle_id, temp_dir)

      if success:
        route_states[route_id] = {'status': 'completed', 'completion_time': time.time()}
      else:
        route_states[route_id] = {'status': 'failed', 'failure_time': time.time(), 'error': 'Upload failed'}
      save_route_states(route_states)

      # Update sync_stats
      if route_id not in sync_stats["routes"]:
        sync_stats["routes"][route_id] = {
          "found": True,
          "uploaded": success,
          "timestamp": route_timestamp,
          "segment_count": len(segments),
          "upload_timestamp": time.time() if success else None,
          "fingerprint": fingerprint,
        }
      else:
        sync_stats["routes"][route_id]["uploaded"] = success
        sync_stats["routes"][route_id]["fingerprint"] = fingerprint
        if success:
          sync_stats["routes"][route_id]["upload_timestamp"] = time.time()

      if success:
        sync_stats["successful_syncs"] += 1
        sync_stats["last_sync_timestamp"] = time.time()
        sync_stats["last_sync_route_id"] = route_id
      else:
        sync_stats["failed_syncs"] += 1

      # Calculate updated status
      uploaded_recent = sum(1 for rid in recent_route_ids if sync_stats["routes"].get(rid, {}).get("uploaded", False))
      total_recent = len(recent_routes)
      remaining = total_recent - uploaded_recent
      print(f"Routes sync status: {uploaded_recent}/{total_recent} recent routes uploaded ({remaining} remaining)")
      save_sync_stats(sync_stats)
      return remaining

    finally:
      release_route_lock(lock_fd, route_id)

  # If no routes were uploaded
  uploaded_recent = sum(1 for rid in recent_route_ids if sync_stats["routes"].get(rid, {}).get("uploaded", False))
  total_recent = len(recent_routes)
  remaining = total_recent - uploaded_recent
  print(f"Routes sync status: {uploaded_recent}/{total_recent} recent routes uploaded ({remaining} remaining)")
  update_sync_status("idle", details={"message": "No new routes to upload"})
  save_sync_stats(sync_stats)
  return remaining


def cleanup_incomplete_uploads(s3_client, s3_config, dongle_id):
  """Identify and clean up incomplete route uploads in S3, respecting the MAX_ROUTE_AGE_DAYS limit."""
  print("Cleaning up incomplete uploads from the last 7 days...")

  try:
    # Calculate the cutoff time (7 days ago)
    cutoff_time = time.time() - (MAX_ROUTE_AGE_DAYS * 24 * 60 * 60)

    # List all objects in the dongle's routes directory
    prefix = f"data_collection/{dongle_id}/routes/"
    paginator = s3_client.get_paginator('list_objects_v2')

    # Group metadata and route files by route_id
    route_files = {}

    # Collect all objects first
    for page in paginator.paginate(Bucket=s3_config["BUCKET_NAME"], Prefix=prefix):
      if 'Contents' not in page:
        continue

      for item in page['Contents']:
        key = item['Key']
        size = item['Size']
        last_modified = item['LastModified'].timestamp() if hasattr(item.get('LastModified', None), 'timestamp') else None
        filename = os.path.basename(key)

        # Skip files older than MAX_ROUTE_AGE_DAYS if we have LastModified timestamp
        if last_modified and last_modified < cutoff_time:
          continue

        if "_metadata.json" in filename:
          route_id = filename.split('_metadata.json')[0]
          if route_id not in route_files:
            route_files[route_id] = {'metadata': {'key': key, 'size': size}}
          else:
            route_files[route_id]['metadata'] = {'key': key, 'size': size}

        elif "_rlog.zst" in filename or "_rlog" in filename:
          route_id = filename.split('_rlog')[0]
          if route_id not in route_files:
            route_files[route_id] = {'rlog': {'key': key, 'size': size}}
          else:
            route_files[route_id]['rlog'] = {'key': key, 'size': size}

    # Find incomplete uploads (only rlog or only metadata)
    incomplete_uploads = []
    for route_id, files in route_files.items():
      if 'metadata' in files and 'rlog' not in files:
        incomplete_uploads.append((route_id, 'metadata', files['metadata']['key']))
      elif 'rlog' in files and 'metadata' not in files:
        incomplete_uploads.append((route_id, 'rlog', files['rlog']['key']))

    # Also check for size mismatches between metadata and actual rlog
    for route_id, files in route_files.items():
      if 'metadata' in files and 'rlog' in files:
        # Download and parse metadata to check expected size and timestamp
        try:
          response = s3_client.get_object(Bucket=s3_config["BUCKET_NAME"], Key=files['metadata']['key'])
          metadata = json.loads(response['Body'].read().decode('utf-8'))

          # Skip routes older than MAX_ROUTE_AGE_DAYS
          if 'timestamp' in metadata and metadata['timestamp'] < cutoff_time:
            continue

          # If metadata contains size info, verify it
          if 'rlog_size' in metadata and metadata['rlog_size'] != files['rlog']['size']:
            print(f"Size mismatch for route {route_id}: expected {metadata['rlog_size']}, found {files['rlog']['size']}")
            incomplete_uploads.append((route_id, 'mismatch', files['rlog']['key']))

          # If metadata contains MD5 info, verify it
          if 'rlog_md5' in metadata:
            try:
              head_response = s3_client.head_object(Bucket=s3_config["BUCKET_NAME"], Key=files['rlog']['key'])
              # For non-multipart uploads, ETag is the MD5 hash (in quotes)
              s3_etag = head_response.get('ETag', '').strip('"')

              if s3_etag != metadata['rlog_md5']:
                print(f"MD5 mismatch for route {route_id}: expected {metadata['rlog_md5']}, found {s3_etag}")
                # Note: This might be expected for multipart uploads
                # For simplicity, we're not handling the complex multipart ETag verification here
            except Exception as e:
              print(f"Error checking ETag for route {route_id}: {e}")

        except Exception as e:
          print(f"Error checking metadata for route {route_id}: {e}")

    # Clean up incomplete uploads
    for route_id, issue_type, key in incomplete_uploads:
      print(f"Cleaning up incomplete upload for route {route_id} (issue: {issue_type})")
      try:
        s3_client.delete_object(Bucket=s3_config["BUCKET_NAME"], Key=key)
        print(f"Deleted incomplete file: {key}")

        # If we deleted metadata, also look for and delete the corresponding rlog file
        if issue_type == 'metadata' and key.endswith('_metadata.json'):
          rlog_key = key.replace('_metadata.json', '_rlog.zst')
          try:
            s3_client.head_object(Bucket=s3_config["BUCKET_NAME"], Key=rlog_key)
            s3_client.delete_object(Bucket=s3_config["BUCKET_NAME"], Key=rlog_key)
            print(f"Also deleted corresponding rlog file: {rlog_key}")
          except ClientError:
            # File might not exist, that's okay
            pass

        # If we deleted rlog, also look for and delete the corresponding metadata file
        if issue_type == 'rlog' and ('_rlog.zst' in key or '_rlog' in key):
          metadata_key = key.replace('_rlog.zst', '_metadata.json').replace('_rlog', '_metadata.json')
          try:
            s3_client.head_object(Bucket=s3_config["BUCKET_NAME"], Key=metadata_key)
            s3_client.delete_object(Bucket=s3_config["BUCKET_NAME"], Key=metadata_key)
            print(f"Also deleted corresponding metadata file: {metadata_key}")
          except ClientError:
            # File might not exist, that's okay
            pass

      except Exception as e:
        print(f"Failed to delete {key}: {e}")

    return len(incomplete_uploads)

  except Exception as e:
    print(f"Failed to clean up incomplete uploads: {e}")
    return 0


def main(exit_event=None):
  """Main function for the bluepilot uploader process."""
  print("BPUploader: Starting uploader for route data")
  if exit_event is None:
    exit_event = threading.Event()

  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("Failed to set core affinity")

  cloudlog.info("BPUploader: Starting uploader for route data")
  update_sync_status("initializing")

  # Initialize variables
  backoff = 0.1
  last_sync_time = 0
  current_sync_interval = SYNC_INTERVAL
  check_interval = 10  # Check for changed conditions every 10 seconds
  startup_refresh_done = False

  # Create necessary directories
  os.makedirs(CACHE_DIR, exist_ok=True)
  os.makedirs(TEMP_LOGS_DIR, exist_ok=True)
  os.makedirs(LOCK_DIR, exist_ok=True)

  # Main loop
  while not exit_event.is_set():
    params = Params()
    upload_enabled = params.get_bool("EnableBluepilotRlogUploader")

    if not upload_enabled:
      print("BPUploader: Uploader is disabled. Checking again later...")
      update_sync_status("idle", details={"message": "Uploader disabled"})
      time.sleep(check_interval)
      continue

    sm = messaging.SubMaster(['deviceState'])
    sm.update(0)

    offroad = params.get_bool("IsOffroad")
    # network_type = sm['deviceState'].networkType if not force_wifi else NetworkType.wifi
    # metered = sm['deviceState'].networkMetered
    # if not network_type == NetworkType.wifi and not network_type == NetworkType.ethernet:
    #   print("BPUploader: Not on WiFi or Ethernet. Waiting for suitable network connection...")
    #   update_sync_status("idle", details={"message": "Waiting for WiFi/Ethernet connection"})
    #   time.sleep(60 if offroad else 5)
    #   continue

    # elif network_type == NetworkType.cell2G or network_type == NetworkType.cell3G or network_type == NetworkType.cell4G or network_type == NetworkType.cell5G:
    #   print("BPUploader: On cellular. Waiting for WiFi/Ethernet connection...")
    #   update_sync_status("idle", details={"message": "Waiting for WiFi/Ethernet connection"})
    #   time.sleep(check_interval)
    #   continue

    # elif metered:
    #   print("BPUploader: Metered connection. Waiting for WiFi/Ethernet connection...")
    #   update_sync_status("idle", details={"message": "Waiting for WiFi/Ethernet connection"})
    #   time.sleep(check_interval)
    #   continue

    # print(
    #   f"BPUploader: Network type: {network_type}, Metered: {metered}, Offroad: {offroad}, On Wifi: {on_wifi}, On Ethernet: {on_ethernet}, On Cell: {on_cell}"
    # )

    if not offroad:
      print("BPUploader: Not offroad. Waiting for offroad...")
      update_sync_status("idle", details={"message": "Waiting for offroad"})
      time.sleep(check_interval)
      continue

    # Only refresh S3 stats once when conditions become suitable for upload
    if not startup_refresh_done:
      try:
        print("BPUploader: Initializing S3 client for startup sync stats refresh")
        s3_client, s3_config = init_s3_client()
        if s3_client is not None:
          dongle_id = params.get("DongleId").decode('utf-8') if params.get("DongleId") else "unknown"
          print("BPUploader: Refreshing sync stats from S3 on startup...")
          refresh_sync_stats_from_s3(s3_client, s3_config, dongle_id)
          sync_stats = load_sync_stats()
          sync_stats["last_s3_refresh"] = time.time()
          save_sync_stats(sync_stats)
          print("BPUploader: Startup sync stats refresh complete")
          startup_refresh_done = True
        else:
          print("BPUploader: Failed to initialize S3 client for startup refresh")
      except Exception as e:
        error(f"BPUploader: Failed to refresh sync stats on startup: {e}", True)

    current_time = time.monotonic()
    update_sync_status("idle", details={"message": "Waiting for sync interval"})

    # Only sync if conditions are met (offroad and enough time has passed)
    if offroad and (current_time - last_sync_time > current_sync_interval):
      try:
        remaining_routes = sync_routes(network_type=sm['deviceState'].networkType.raw, metered=sm['deviceState'].networkMetered)
        last_sync_time = current_time
        cloudlog.info("Route data sync completed")
        backoff = 0.1

        if remaining_routes > 0:
          current_sync_interval = 30
          print(f"BPUploader: Next sync in {current_sync_interval} seconds ({remaining_routes} routes remaining)")
          update_sync_status(
            "idle", details={"message": "Waiting for next sync", "next_sync": time.time() + current_sync_interval, "remaining_routes": remaining_routes}
          )
        else:
          current_sync_interval = SYNC_INTERVAL
          print(f"BPUploader: All recent routes synced. Next sync in {SYNC_INTERVAL} seconds")
          update_sync_status("idle", details={"message": "All recent routes synced", "next_sync": time.time() + SYNC_INTERVAL})
      except Exception as e:
        cloudlog.exception(f"BPUploader: Failed to sync route data: {e}")
        backoff = min(backoff * 2, 60)
        update_sync_status("error", details={"message": f"Error during sync: {e}", "retry_in": backoff})

    # Sleep for a short time, then check conditions again
    sleep_time = min(check_interval, backoff + random.uniform(0, backoff))
    time.sleep(sleep_time)


if __name__ == "__main__":
  main()
