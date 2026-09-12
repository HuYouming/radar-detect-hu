"""Serialization for native vision and ray-casting calls."""

import threading


# PyTorch/Ultralytics and Open3D both use native thread pools. Serializing
# entry into their native operations avoids intermittent aborts in constrained
# container deployments while keeping Python-side pipeline threads intact.
NATIVE_COMPUTE_LOCK = threading.RLock()
