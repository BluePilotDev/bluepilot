"""The portal's log file. Kept in a module that imports nothing else from the portal, so bp_portal can set up
logging before any other portal module is imported (some log at import)."""
import logging
import logging.handlers


def portal_file_handler(log_file):
    """The portal's log file: 2 MB, one old copy kept"""
    handler = logging.handlers.RotatingFileHandler(log_file, maxBytes=2 * 1024 * 1024, backupCount=1)
    handler.setLevel(logging.INFO)
    handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s [%(name)s]: %(message)s'))
    return handler
