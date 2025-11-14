#!/usr/bin/env python3
"""
Export & Backup Module for BluePilot Web Server
Handles videos ZIP, route backups, and import/restore functionality
"""

import os
import json
import time
import shutil
import zipfile
import logging
import threading
from datetime import datetime, timezone

logger = logging.getLogger(__name__)


# ============================================================================
# HTTP Handler Delegation Functions
# ============================================================================

def handle_videos_zip_post(handler, path, server_state, get_route_segments,
                          camera_files_dict, generate_route_export,
                          generate_route_export_filename, videos_zip_cache,
                          route_export_cache, get_disk_space_info, broadcast_websocket_event,
                          websocket_event_enum):
    """Handle POST /api/videos-zip/{route_base}"""
    parts = path.split('/')[3:]
    if len(parts) < 1:
        handler.send_json_response({'error': 'Invalid videos-zip path'}, 400)
        return

    route_base = parts[0]
    segments = get_route_segments(route_base)
    if not segments:
        handler.send_json_response({'error': 'Route not found'}, 404)
        return

    # Parse request body
    try:
        content_length = int(handler.headers.get('Content-Length', 0))
        body = handler.rfile.read(content_length).decode('utf-8')
        data = json.loads(body)
        cameras = data.get('cameras', [])
    except Exception as e:
        handler.send_json_response({'error': f'Invalid request body: {e}'}, 400)
        return

    if not cameras:
        handler.send_json_response({'error': 'No cameras specified'}, 400)
        return

    # Start videos ZIP creation
    started = server_state.start_videos_zip(route_base, "Creating videos archive")
    if not started:
        # Already in progress
        status = server_state.get_videos_zip_status(route_base)
        handler.send_json_response(status)
        return

    def progress_callback(progress, message):
        info = server_state.update_videos_zip(route_base, progress=progress, message=message)
        # Broadcast via WebSocket for real-time updates
        broadcast_websocket_event(websocket_event_enum.EXPORT_PROGRESS, {
            'type': 'videos_zip',
            'route_base': route_base,
            'progress': progress,
            'message': message,
            'status': info.get('status', 'processing')
        })

    def worker():
        try:
            zip_path = create_videos_zip(
                route_base, cameras, segments, camera_files_dict,
                videos_zip_cache, route_export_cache,
                get_disk_space_info, generate_route_export,
                generate_route_export_filename, progress_callback, server_state
            )
            info = server_state.complete_videos_zip(route_base, zip_path, "ZIP ready")
            # Broadcast completion
            broadcast_websocket_event(websocket_event_enum.EXPORT_COMPLETE, {
                'type': 'videos_zip',
                'route_base': route_base,
                'status': 'ready',
                'message': 'ZIP archive ready'
            })
        except Exception as exc:
            logger.error(f"Videos ZIP failed for {route_base}: {exc}", exc_info=True)
            server_state.fail_videos_zip(route_base, str(exc))
            # Broadcast error
            broadcast_websocket_event(websocket_event_enum.EXPORT_ERROR, {
                'type': 'videos_zip',
                'route_base': route_base,
                'error': str(exc)
            })

    thread = threading.Thread(target=worker, daemon=True, name=f"videos-zip-{route_base}")
    thread.start()

    status = server_state.get_videos_zip_status(route_base)
    handler.send_json_response(status)


def handle_route_backup_post(handler, path, server_state, get_route_segments,
                            backup_cache, metrics_cache, thumbnail_cache,
                            get_disk_space_info, broadcast_websocket_event,
                            websocket_event_enum):
    """Handle POST /api/route-backup/{route_base}"""
    parts = path.split('/')[3:]
    if len(parts) < 1:
        handler.send_json_response({'error': 'Invalid backup path'}, 400)
        return

    route_base = parts[0]
    segments = get_route_segments(route_base)
    if not segments:
        handler.send_json_response({'error': 'Route not found'}, 404)
        return

    # Start backup creation
    started = server_state.start_backup(route_base, "Creating backup")
    if not started:
        status = server_state.get_backup_status(route_base)
        handler.send_json_response(status)
        return

    def progress_callback(progress, message):
        info = server_state.update_backup(route_base, progress=progress, message=message)
        # Broadcast via WebSocket
        broadcast_websocket_event(websocket_event_enum.EXPORT_PROGRESS, {
            'type': 'route_backup',
            'route_base': route_base,
            'progress': progress,
            'message': message,
            'status': info.get('status', 'processing')
        })

    def worker():
        try:
            backup_path = create_route_backup(
                route_base, segments, backup_cache, metrics_cache,
                thumbnail_cache, get_disk_space_info, progress_callback
            )
            info = server_state.complete_backup(route_base, backup_path, "Backup ready")
            # Broadcast completion
            broadcast_websocket_event(websocket_event_enum.EXPORT_COMPLETE, {
                'type': 'route_backup',
                'route_base': route_base,
                'status': 'ready',
                'message': 'Backup ready'
            })
        except Exception as exc:
            logger.error(f"Backup failed for {route_base}: {exc}", exc_info=True)
            server_state.fail_backup(route_base, str(exc))
            # Broadcast error
            broadcast_websocket_event(websocket_event_enum.EXPORT_ERROR, {
                'type': 'route_backup',
                'route_base': route_base,
                'error': str(exc)
            })

    thread = threading.Thread(target=worker, daemon=True, name=f"backup-{route_base}")
    thread.start()

    status = server_state.get_backup_status(route_base)
    handler.send_json_response(status)


def handle_route_import_post(handler, path, server_state, import_temp_dir,
                            routes_dir, metrics_cache, thumbnail_cache,
                            set_route_preserve, broadcast_websocket_event,
                            websocket_event_enum):
    """Handle POST /api/route-import"""
    import cgi

    # Parse multipart form data
    try:
        content_type = handler.headers.get('Content-Type', '')
        if 'multipart/form-data' not in content_type:
            handler.send_json_response({'error': 'Expected multipart/form-data'}, 400)
            return

        # Parse form data
        form = cgi.FieldStorage(
            fp=handler.rfile,
            headers=handler.headers,
            environ={
                'REQUEST_METHOD': 'POST',
                'CONTENT_TYPE': content_type,
            }
        )

        if 'backup' not in form:
            handler.send_json_response({'error': 'No backup file provided'}, 400)
            return

        file_item = form['backup']
        if not file_item.file:
            handler.send_json_response({'error': 'Invalid file upload'}, 400)
            return

        # Generate unique import ID
        import_id = f"{int(time.time())}_{os.urandom(4).hex()}"

        # Save uploaded file
        upload_path = os.path.join(import_temp_dir, f"{import_id}.zip")
        with open(upload_path, 'wb') as f:
            shutil.copyfileobj(file_item.file, f)

        # Start import operation
        started = server_state.start_import(import_id, "Uploading backup")
        if not started:
            handler.send_json_response({'error': 'Import already in progress'}, 409)
            return

        def progress_callback(progress, message):
            info = server_state.update_import(import_id, progress=progress, message=message)
            # Broadcast via WebSocket
            broadcast_websocket_event(websocket_event_enum.EXPORT_PROGRESS, {
                'type': 'route_import',
                'import_id': import_id,
                'progress': progress,
                'message': message,
                'status': info.get('status', 'processing')
            })

        def worker():
            try:
                route_name = import_route_backup(
                    upload_path, import_id, import_temp_dir, routes_dir,
                    metrics_cache, thumbnail_cache, set_route_preserve, progress_callback
                )
                info = server_state.complete_import(import_id, route_name, "Import complete")
                # Broadcast completion
                broadcast_websocket_event(websocket_event_enum.EXPORT_COMPLETE, {
                    'type': 'route_import',
                    'import_id': import_id,
                    'route_name': route_name,
                    'status': 'completed',
                    'message': 'Import complete'
                })
            except Exception as exc:
                logger.error(f"Import failed for {import_id}: {exc}", exc_info=True)
                server_state.fail_import(import_id, str(exc))
                # Broadcast error
                broadcast_websocket_event(websocket_event_enum.EXPORT_ERROR, {
                    'type': 'route_import',
                    'import_id': import_id,
                    'error': str(exc)
                })
            finally:
                # Clean up uploaded file
                try:
                    os.remove(upload_path)
                except:
                    pass

        thread = threading.Thread(target=worker, daemon=True, name=f"import-{import_id}")
        thread.start()

        handler.send_json_response({'success': True, 'importId': import_id})
        return

    except Exception as e:
        logger.error(f"Error handling route import: {e}", exc_info=True)
        handler.send_json_response({'error': f'Upload failed: {e}'}, 500)
        return


def handle_videos_zip_status_get(handler, path, server_state):
    """Handle GET /api/videos-zip/{route_base}/status"""
    route_base = path.split('/')[3]
    status = server_state.get_videos_zip_status(route_base)
    if status:
        handler.send_json_response(status)
    else:
        handler.send_json_response({'status': 'idle', 'progress': 0})


def handle_videos_zip_download_get(handler, path, server_state):
    """Handle GET /api/videos-zip/{route_base}/download"""
    route_base = path.split('/')[3]
    status = server_state.get_videos_zip_status(route_base)

    if not status or status.get('status') != 'ready':
        handler.send_json_response({'error': 'ZIP not ready'}, 404)
        return

    zip_path = status.get('path')
    if not zip_path or not os.path.exists(zip_path):
        handler.send_json_response({'error': 'ZIP file not found'}, 404)
        return

    # Serve the file
    handler.send_file_response(zip_path)

    # Schedule cleanup after download (1 minute delay to ensure download completes)
    def cleanup():
        time.sleep(60)
        try:
            if os.path.exists(zip_path):
                os.remove(zip_path)
                logger.info(f"Cleaned up videos ZIP: {zip_path}")
            server_state.clear_videos_zip(route_base)
        except Exception as e:
            logger.warning(f"Failed to cleanup videos ZIP {zip_path}: {e}")

    threading.Thread(target=cleanup, daemon=True, name=f"cleanup-videos-zip-{route_base}").start()


def handle_route_backup_status_get(handler, path, server_state):
    """Handle GET /api/route-backup/{route_base}/status"""
    route_base = path.split('/')[3]
    status = server_state.get_backup_status(route_base)
    if status:
        handler.send_json_response(status)
    else:
        handler.send_json_response({'status': 'idle', 'progress': 0})


def handle_route_backup_download_get(handler, path, server_state):
    """Handle GET /api/route-backup/{route_base}/download"""
    route_base = path.split('/')[3]
    status = server_state.get_backup_status(route_base)

    if not status or status.get('status') != 'ready':
        handler.send_json_response({'error': 'Backup not ready'}, 404)
        return

    backup_path = status.get('path')
    if not backup_path or not os.path.exists(backup_path):
        handler.send_json_response({'error': 'Backup file not found'}, 404)
        return

    # Serve the file
    handler.send_file_response(backup_path)

    # Schedule cleanup after download (1 minute delay to ensure download completes)
    def cleanup():
        time.sleep(60)
        try:
            if os.path.exists(backup_path):
                os.remove(backup_path)
                logger.info(f"Cleaned up route backup: {backup_path}")
            server_state.clear_backup(route_base)
        except Exception as e:
            logger.warning(f"Failed to cleanup backup {backup_path}: {e}")

    threading.Thread(target=cleanup, daemon=True, name=f"cleanup-backup-{route_base}").start()


def handle_route_import_status_get(handler, path, server_state):
    """Handle GET /api/route-import/{import_id}/status"""
    import_id = path.split('/')[3]
    status = server_state.get_import_status(import_id)
    if status:
        handler.send_json_response(status)
    else:
        handler.send_json_response({'status': 'idle', 'progress': 0})


# ============================================================================
# Helper Functions
# ============================================================================

def create_videos_zip(route_base, cameras, segments, camera_files_dict,
                     videos_zip_cache, route_export_cache,
                     get_disk_space_info, generate_route_export,
                     generate_route_export_filename, progress_callback=None, server_state=None):
    """
    Create a ZIP file containing exported videos for multiple cameras

    Args:
        route_base: Route identifier (e.g., "2024-09-18--14-30-00")
        cameras: List of camera names (e.g., ['front', 'wide'])
        segments: List of segment dictionaries
        camera_files_dict: CAMERA_FILES mapping
        videos_zip_cache: Path to ZIP cache directory
        route_export_cache: Path to route export cache
        get_disk_space_info: Function to get disk space info
        generate_route_export: Function to generate individual camera export
        generate_route_export_filename: Function to generate export filename
        progress_callback: Optional callback(progress, message)
        server_state: Server state instance for FFmpeg process management

    Returns:
        Path to the created ZIP file
    """
    zip_path = os.path.join(videos_zip_cache, f"{route_base}_videos.zip")

    if progress_callback:
        progress_callback(0.01, "Checking disk space...")

    # Estimate required disk space (sum of all camera export sizes)
    estimated_size = 0
    for camera in cameras:
        if camera not in camera_files_dict:
            continue
        for segment in segments:
            video_file = os.path.join(segment['path'], camera_files_dict[camera])
            if os.path.exists(video_file):
                estimated_size += os.path.getsize(video_file)

    # Add 20% overhead for ZIP file + temporary exports
    required_space = int(estimated_size * 1.2)

    # Check available disk space
    disk_info = get_disk_space_info()
    available_bytes = disk_info.get('available_bytes', 0)

    if required_space > available_bytes:
        required_gb = required_space / (1024**3)
        available_gb = available_bytes / (1024**3)
        raise ValueError(
            f"Insufficient disk space. Required: {required_gb:.2f} GB, "
            f"Available: {available_gb:.2f} GB"
        )

    if progress_callback:
        progress_callback(0.05, "Preparing video exports...")

    # Generate exports for each camera
    camera_files = {}
    total_cameras = len(cameras)

    for idx, camera in enumerate(cameras):
        if camera not in camera_files_dict:
            logger.warning(f"Invalid camera type: {camera}, skipping")
            continue

        try:
            if progress_callback:
                progress_pct = 0.1 + (0.65 * idx / total_cameras)
                progress_callback(progress_pct, f"Exporting {camera} camera...")

            # Generate individual video export
            video_path = generate_route_export(route_base, camera, progress_callback, server_state)
            camera_files[camera] = video_path

        except Exception as e:
            logger.error(f"Failed to export {camera} camera: {e}")
            # Continue with other cameras

    if not camera_files:
        raise ValueError("No videos could be exported")

    if progress_callback:
        progress_callback(0.80, "Compressing videos into ZIP...")

    # Create ZIP file with compression
    total_files = len(camera_files)
    logger.info(f"Creating videos ZIP with {total_files} camera files")

    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as zipf:
        for idx, (camera, video_path) in enumerate(camera_files.items()):
            if os.path.exists(video_path):
                # Use descriptive filename in ZIP
                filename = generate_route_export_filename(route_base, camera, segments)
                file_size_mb = os.path.getsize(video_path) / (1024 * 1024)

                # Show progress before writing (especially important for large files)
                if progress_callback:
                    progress_pct = 0.80 + (0.18 * idx / total_files)
                    progress_callback(progress_pct, f"Compressing {camera} ({file_size_mb:.1f}MB)... {idx + 1}/{total_files}")

                logger.debug(f"Adding to ZIP: {camera} camera ({file_size_mb:.1f}MB)")
                zipf.write(video_path, filename)

    if progress_callback:
        progress_callback(1.0, "ZIP archive ready")

    final_size_mb = os.path.getsize(zip_path) / (1024 * 1024)
    logger.info(f"Videos ZIP created successfully: {zip_path} ({final_size_mb:.1f}MB)")

    return zip_path


def create_route_backup(route_base, segments, backup_cache, metrics_cache,
                       thumbnail_cache, get_disk_space_info, progress_callback=None):
    """
    Create a complete backup of a route including all segments, videos, logs, and metadata

    Args:
        route_base: Route identifier
        segments: List of segment dictionaries
        backup_cache: Path to backup cache directory
        metrics_cache: Path to metrics cache directory
        thumbnail_cache: Path to thumbnail cache directory
        get_disk_space_info: Function to get disk space info
        progress_callback: Optional callback(progress, message)

    Returns:
        Path to the created backup ZIP file
    """
    backup_path = os.path.join(backup_cache, f"{route_base}.bpbackup.zip")

    if progress_callback:
        progress_callback(0.01, "Checking disk space...")

    # Calculate total size of all segment files
    total_size = 0
    for segment in segments:
        segment_dir = segment['path']
        if os.path.exists(segment_dir):
            for filename in os.listdir(segment_dir):
                file_path = os.path.join(segment_dir, filename)
                if os.path.isfile(file_path):
                    total_size += os.path.getsize(file_path)

    # Add overhead for ZIP compression (estimate 10% overhead)
    required_space = int(total_size * 1.1)

    # Check available disk space
    disk_info = get_disk_space_info()
    available_bytes = disk_info.get('available_bytes', 0)

    if required_space > available_bytes:
        required_gb = required_space / (1024**3)
        available_gb = available_bytes / (1024**3)
        raise ValueError(
            f"Insufficient disk space for backup. Required: {required_gb:.2f} GB, "
            f"Available: {available_gb:.2f} GB"
        )

    if progress_callback:
        progress_callback(0.02, "Scanning route segments...")

    total_segments = len(segments)

    # Collect all files to backup
    files_to_backup = []

    # Add segment files
    for idx, segment in enumerate(segments):
        segment_dir = segment['path']
        segment_num = segment['segment']

        # Add all files in segment directory
        if os.path.exists(segment_dir):
            for filename in os.listdir(segment_dir):
                file_path = os.path.join(segment_dir, filename)
                if os.path.isfile(file_path):
                    # Store relative path within route
                    rel_path = f"segments/{route_base}--{segment_num}/{filename}"
                    files_to_backup.append((file_path, rel_path))

        if progress_callback:
            progress_pct = 0.05 + (0.3 * idx / total_segments)
            progress_callback(progress_pct, f"Scanning segment {segment_num + 1}/{total_segments}...")

    if progress_callback:
        progress_callback(0.35, "Collecting metadata...")

    # Collect cached metadata
    metadata_files = {}

    # GPS metrics
    metrics_file = os.path.join(metrics_cache, f"{route_base}.json")
    if os.path.exists(metrics_file):
        metadata_files['metrics.json'] = metrics_file

    # Thumbnail
    thumb_file = os.path.join(thumbnail_cache, f"{route_base}.jpg")
    if os.path.exists(thumb_file):
        metadata_files['thumbnail.jpg'] = thumb_file

    # Fingerprint
    fingerprint_file = os.path.join(os.path.dirname(metrics_cache), "fingerprint_cache", f"{route_base}.json")
    if os.path.exists(fingerprint_file):
        metadata_files['fingerprint.json'] = fingerprint_file

    # Drive stats
    stats_file = os.path.join(os.path.dirname(metrics_cache), "drive_stats_cache", f"{route_base}.json")
    if os.path.exists(stats_file):
        metadata_files['drive_stats.json'] = stats_file

    # Create manifest
    manifest = {
        'route_base': route_base,
        'backup_version': '1.0',
        'created_at': datetime.now(timezone.utc).isoformat(),
        'segments_count': total_segments,
        'segments': [
            {
                'number': seg['segment'],
                'timestamp': seg.get('timestamp', '')
            }
            for seg in segments
        ]
    }

    if progress_callback:
        progress_callback(0.4, "Creating backup archive...")

    # Create ZIP file
    with zipfile.ZipFile(backup_path, 'w', zipfile.ZIP_STORED) as zipf:
        # Add manifest
        zipf.writestr('manifest.json', json.dumps(manifest, indent=2))

        # Add metadata files
        for rel_path, abs_path in metadata_files.items():
            zipf.write(abs_path, f"cache/{rel_path}")

        # Add segment files with progress
        total_files = len(files_to_backup)
        logger.info(f"Starting backup archive creation with {total_files} files")

        for idx, (abs_path, rel_path) in enumerate(files_to_backup):
            try:
                # Show progress before writing (especially important for large files)
                if progress_callback:
                    progress_pct = 0.4 + (0.6 * idx / total_files)
                    filename = os.path.basename(rel_path)
                    file_size_mb = os.path.getsize(abs_path) / (1024 * 1024) if os.path.exists(abs_path) else 0

                    # Extract segment number from path for better progress messages
                    # Path format: segments/{route_base}--{segment_num}/{filename}
                    segment_info = ""
                    if '--' in rel_path:
                        try:
                            segment_part = rel_path.split('/')[-2]  # Get the directory name
                            segment_num = segment_part.split('--')[-1]
                            segment_info = f"Segment {segment_num}/{total_segments - 1} - "
                        except:
                            pass

                    progress_callback(progress_pct, f"{segment_info}{filename} ({file_size_mb:.1f}MB) - {idx + 1}/{total_files}")

                logger.debug(f"Adding to backup: {rel_path} ({file_size_mb:.1f}MB)")
                zipf.write(abs_path, rel_path)

            except Exception as e:
                logger.error(f"Failed to add file to backup: {rel_path}: {e}", exc_info=True)

    if progress_callback:
        progress_callback(1.0, "Backup complete")

    final_size_mb = os.path.getsize(backup_path) / (1024 * 1024)
    logger.info(f"Route backup created successfully: {backup_path} ({final_size_mb:.1f}MB, {total_files} files)")

    return backup_path


def import_route_backup(backup_file_path, import_id, import_temp_dir, routes_dir,
                       metrics_cache, thumbnail_cache, set_route_preserve, progress_callback=None):
    """
    Import a route backup and restore it to the routes directory

    Args:
        backup_file_path: Path to the uploaded backup ZIP file
        import_id: Unique identifier for this import operation
        import_temp_dir: Path to temporary import directory
        routes_dir: Path to routes directory
        metrics_cache: Path to metrics cache directory
        thumbnail_cache: Path to thumbnail cache directory
        set_route_preserve: Function to preserve a route
        progress_callback: Optional callback(progress, message)

    Returns:
        Route base name of the imported route
    """
    if progress_callback:
        progress_callback(0.01, "Validating backup file...")

    # Verify it's a valid ZIP
    if not zipfile.is_zipfile(backup_file_path):
        raise ValueError("Invalid backup file: not a ZIP archive")

    # Create temp extraction directory
    extract_dir = os.path.join(import_temp_dir, import_id)
    os.makedirs(extract_dir, exist_ok=True)

    try:
        # Extract backup
        if progress_callback:
            progress_callback(0.05, "Extracting backup...")

        with zipfile.ZipFile(backup_file_path, 'r') as zipf:
            zipf.extractall(extract_dir)

        # Load and validate manifest
        manifest_path = os.path.join(extract_dir, 'manifest.json')
        if not os.path.exists(manifest_path):
            raise ValueError("Invalid backup: missing manifest.json")

        with open(manifest_path, 'r') as f:
            manifest = json.load(f)

        route_base = manifest.get('route_base')
        if not route_base:
            raise ValueError("Invalid backup: missing route_base in manifest")

        if progress_callback:
            progress_callback(0.1, f"Importing route {route_base}...")

        # Restore segment files
        segments_dir = os.path.join(extract_dir, 'segments')
        if os.path.exists(segments_dir):
            segment_count = manifest.get('segments_count', 0)
            processed = 0

            for segment_dir_name in os.listdir(segments_dir):
                segment_dir = os.path.join(segments_dir, segment_dir_name)
                if not os.path.isdir(segment_dir):
                    continue

                # Extract segment number for progress message
                segment_num = "?"
                if '--' in segment_dir_name:
                    try:
                        segment_num = segment_dir_name.split('--')[-1]
                    except:
                        pass

                # Show progress before copying
                if progress_callback and segment_count > 0:
                    progress_pct = 0.1 + (0.8 * processed / segment_count)
                    progress_callback(progress_pct, f"Restoring segment {segment_num}/{segment_count - 1}... ({processed + 1}/{segment_count})")

                # Copy to routes directory
                dest_dir = os.path.join(routes_dir, segment_dir_name)
                if os.path.exists(dest_dir):
                    logger.warning(f"Segment {segment_dir_name} already exists, skipping")
                    processed += 1
                    continue

                shutil.copytree(segment_dir, dest_dir)
                processed += 1

        # Restore cached metadata
        cache_dir = os.path.join(extract_dir, 'cache')
        if os.path.exists(cache_dir):
            if progress_callback:
                progress_callback(0.9, "Restoring metadata...")

            # Metrics
            metrics_src = os.path.join(cache_dir, 'metrics.json')
            if os.path.exists(metrics_src):
                shutil.copy(metrics_src, os.path.join(metrics_cache, f"{route_base}.json"))

            # Thumbnail
            thumb_src = os.path.join(cache_dir, 'thumbnail.jpg')
            if os.path.exists(thumb_src):
                shutil.copy(thumb_src, os.path.join(thumbnail_cache, f"{route_base}.jpg"))

            # Fingerprint
            fp_src = os.path.join(cache_dir, 'fingerprint.json')
            if os.path.exists(fp_src):
                fp_cache = os.path.join(os.path.dirname(metrics_cache), "fingerprint_cache")
                os.makedirs(fp_cache, exist_ok=True)
                shutil.copy(fp_src, os.path.join(fp_cache, f"{route_base}.json"))

            # Drive stats
            stats_src = os.path.join(cache_dir, 'drive_stats.json')
            if os.path.exists(stats_src):
                stats_cache = os.path.join(os.path.dirname(metrics_cache), "drive_stats_cache")
                os.makedirs(stats_cache, exist_ok=True)
                shutil.copy(stats_src, os.path.join(stats_cache, f"{route_base}.json"))

        # Preserve the imported route automatically
        if progress_callback:
            progress_callback(0.95, "Preserving route...")

        set_route_preserve(route_base, True)

        if progress_callback:
            progress_callback(1.0, f"Import complete: {route_base}")

        return route_base

    finally:
        # Clean up temp directory
        try:
            shutil.rmtree(extract_dir, ignore_errors=True)
        except:
            pass
