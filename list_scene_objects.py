"""
Diagnostic: lists every top-level object in the current CoppeliaSim scene,
so you can see the exact alias/path to use for the UR5 (e.g. 'UR5' vs
'UR5#0') if sim.getObject('/UR5') is failing.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")

top_level = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 1)  # 1 = direct children only

print(f"Found {len(top_level)} top-level object(s) in the scene:")
for handle in top_level:
    alias = sim.getObjectAlias(handle, 0)
    full_path = sim.getObjectAlias(handle, 1)  # includes full path
    print(f"  - alias='{alias}'  path='{full_path}'")
