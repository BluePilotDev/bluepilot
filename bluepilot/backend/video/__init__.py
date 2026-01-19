"""
BluePilot Backend Video Module
Video processing, streaming, and export functionality
"""

from .ffmpeg import FFmpegProcess, stream_ffmpeg_logs
from .metadata import (
    get_video_duration_from_cache,
    get_video_duration,
    get_video_files,
    get_log_files,
)
from .export import (
    route_export_key,
    get_export_output_path,
    generate_route_export_filename,
    export_is_up_to_date,
    format_route_export_status,
    broadcast_route_export_update,
    generate_route_export,
    stream_route_export,
)
from .remux import (
    remux_segment_to_cache,
    prefetch_next_segments,
)

__all__ = [
    # FFmpeg process management
    'FFmpegProcess',
    'stream_ffmpeg_logs',
    # Metadata
    'get_video_duration_from_cache',
    'get_video_duration',
    'get_video_files',
    'get_log_files',
    # Export
    'route_export_key',
    'get_export_output_path',
    'generate_route_export_filename',
    'export_is_up_to_date',
    'format_route_export_status',
    'broadcast_route_export_update',
    'generate_route_export',
    'stream_route_export',
    # Remux
    'remux_segment_to_cache',
    'prefetch_next_segments',
]
