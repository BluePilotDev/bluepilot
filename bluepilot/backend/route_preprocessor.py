"""
Compatibility shim for moved module

This module was moved to bluepilot.backend.routes.preprocessor during modularization.
This file provides backward compatibility for any code still importing from the old location.

DEPRECATED: Import from bluepilot.backend.routes.preprocessor instead
"""

# Re-export everything from the new location
from bluepilot.backend.routes.preprocessor import *  # noqa: F401, F403

# Support running as a script
if __name__ == "__main__":
    from bluepilot.backend.routes.preprocessor import main
    main()
