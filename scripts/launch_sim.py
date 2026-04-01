import os
import argparse
import collections
import time
import yaml
try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

def main():
    parser = argparse.ArgumentParser(description="Launch IsaacSim with specified assets")
    parser.add_argument(
        "--config", 
        type=str, 
        default=os.path.join(os.path.dirname(__file__), "configs", "simulation_config.yaml"), 
        help="Path to configuration file"
    )
    args = parser.parse_args()

    config_path = os.path.abspath(args.config)
    print(f"Loading configuration from: {config_path}")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Resolve paths relative to the repository root
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    
    env_asset_path = os.path.join(repo_root, config.get("environment_asset", ""))
    print(f"Environment asset: {env_asset_path}")

    # Parse network configuration
    network_setup = config.get("network_setup", {})
    ros2_domain_id = network_setup.get("ros2_domain_id", 0)
    use_discovery_server = network_setup.get("use_discovery_server", False)
    discovery_server_address = network_setup.get("discovery_server_address", "127.0.0.1:11811")
    force_tcp_transport = network_setup.get("force_tcp_transport", False)

    # Set ROS_DOMAIN_ID environment variable to ensure IsaacSim ROS2 bridge communicates on the right domain
    os.environ["ROS_DOMAIN_ID"] = str(ros2_domain_id)
    print(f"Configured ROS_DOMAIN_ID: {ros2_domain_id}")

    # Set Hardware in the Loop (TCP/IP) variables before importing SimulationApp
    if use_discovery_server:
        os.environ["ROS_DISCOVERY_SERVER"] = discovery_server_address
        print(f"Configured ROS_DISCOVERY_SERVER: {discovery_server_address}")
        
    if force_tcp_transport:
        fastdds_profile_path = os.path.join(os.path.dirname(__file__), "configs", "fastdds_tcp.xml")
        if os.path.exists(fastdds_profile_path):
            os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = os.path.abspath(fastdds_profile_path)
            print(f"Configured FASTRTPS_DEFAULT_PROFILES_FILE using strict TCP profile: {fastdds_profile_path}")
        else:
            print(f"Warning: force_tcp_transport is enabled but profile was not found at {fastdds_profile_path}")

    # Initialize the simulation app
    viewport_opts = config.get("viewport_settings", {})
    render_res = viewport_opts.get("render_resolution", [2560, 1440])

    # This must happen before other omni imports.
    # Load the full Isaac Sim experience (same kit config as isaac-sim.sh) so all
    # panels, editors, and extensions are present — not the minimal Python standalone UI.
    # CARB_APP_PATH points to .../release/kit; the experience kit files live in the
    # sibling .../release/apps/ directory, so step up one level.
    _carb_app_path = os.environ.get("CARB_APP_PATH", "")
    _release_dir = os.path.dirname(_carb_app_path)  # .../release
    _exp_full = os.path.join(_release_dir, "apps", "isaacsim.exp.full.kit")
    if not os.path.exists(_exp_full):
        # Fallback: search apps/ for any isaacsim full kit file
        _apps_dir = os.path.join(_release_dir, "apps")
        _candidates = [f for f in os.listdir(_apps_dir) if "full" in f and f.endswith(".kit")] if os.path.isdir(_apps_dir) else []
        if _candidates:
            _exp_full = os.path.join(_apps_dir, sorted(_candidates)[0])
            print(f"[SimApp] Using experience: {_exp_full}")
        else:
            print(f"[SimApp] Warning: isaacsim.exp.full.kit not found in {_apps_dir}, falling back to default")
            _exp_full = ""
    simulation_app = SimulationApp({
        "headless": False,
        "experience": _exp_full,
        "width": render_res[0],
        "height": render_res[1],
        "display_options": 3287  # 3286 (default) | 1 (DISP_FPS)
    })
    
    # Force output renderer resolution specifically (useful if decoupled from window size)
    import carb
    carb_settings = carb.settings.get_settings()
    carb_settings.set_int("/app/renderer/resolution/width", render_res[0])
    carb_settings.set_int("/app/renderer/resolution/height", render_res[1])
    
    # Toggle viewport FPS display
    carb_settings.set_bool("/app/window/showFps", True)
    carb_settings.set_bool("/app/viewport/showFps", True)
    carb_settings.set_bool("/exts/omni.kit.viewport.window/fps", True)

    # DLSS Super Resolution + Frame Generation (FPS Multiplier x2)
    # /rtx/post/aa/op: 0=None,1=TAA,2=FXAA,3=DLSS,4=DLAA
    # /rtx/post/dlss/execMode: 0=Performance(~2x), 1=Balanced, 2=Quality, 3=UltraPerf
    # /rtx-transient/dlssg/enabled: DLSS-G frame generation = FPS Multiplier in the UI
    if viewport_opts.get("enable_DLSS_FPS_Multiplier_x2", False):
        carb_settings.set_int("/rtx/post/aa/op", 3)
        carb_settings.set_int("/rtx/post/dlss/execMode", 0)
        carb_settings.set_bool("/rtx-transient/dlssg/enabled", True)
        print("[Renderer] DLSS Performance + FPS Multiplier x2 (DLSS-G) enabled.")

    # Prevent Isaac Sim Full from auto-adding a defaultLight to the stage.
    # The environment USD already contains a DomeLight; a second light would alter the scene.
    carb_settings.set_bool("/app/stage/generateDefaultLight", False)

    print(f"Configured Viewport Render Resolution to {render_res[0]}x{render_res[1]} with FPS counter")

    import omni.usd
    from pxr import UsdGeom, Gf
    import omni.ext

    # Ensure a stage is open
    context = omni.usd.get_context()
    if not context.get_stage():
        context.new_stage()
    stage = context.get_stage()
    
    def strip_embed_physics_scenes(filepath):
        from pxr import Usd, UsdPhysics
        try:
            temp_stage = Usd.Stage.Open(filepath)
            scenes = [p.GetPath() for p in temp_stage.Traverse() if p.IsA(UsdPhysics.Scene)]
            if scenes:
                for path in scenes:
                    temp_stage.RemovePrim(path)
                temp_stage.Save()
                print(f"[Physics] Autocleaned embedded PhysicsScene natively from {filepath}")
        except Exception:
            pass
            
    # Load environment
    env_config = config.get("environment", {})
    if isinstance(env_config, str):
        env_asset_path = env_config
        env_scale = [1.0, 1.0, 1.0]
        env_rot = [0.0, 0.0, 0.0]
        env_pos = [0.0, 0.0, 0.0]
    else:
        env_asset_path = env_config.get("asset", config.get("environment_asset", "assets/environments/pumptrack_simple.usd"))
        env_scale = env_config.get("scale", [1.0, 1.0, 1.0])
        env_rot = env_config.get("rotation_euler", [0.0, 0.0, 0.0])
        env_pos = env_config.get("translation", [0.0, 0.0, 0.0])
        
    env_asset_path = os.path.abspath(os.path.join(repo_root, env_asset_path))
    
    if os.path.exists(env_asset_path):
        strip_embed_physics_scenes(env_asset_path)
        env_prim = stage.DefinePrim("/World/Environment", "Xform")
        
        # Apply transformations using UsdGeom Xformable APIs securely
        xformable = UsdGeom.Xformable(env_prim)
        xformable.AddScaleOp().Set(Gf.Vec3d(*env_scale))
        xformable.AddRotateXYZOp().Set(Gf.Vec3d(*env_rot))
        xformable.AddTranslateOp().Set(Gf.Vec3d(*env_pos))
        
        env_prim.GetReferences().AddReference(env_asset_path)
        print(f"Successfully added environment reference to stage with Scale {env_scale} and Rotation {env_rot}.")
    else:
        print(f"Warning: Environment asset not found at {env_asset_path}")

    # Load vehicles
    vehicles = config.get("vehicles", [])
    for veh in vehicles:
        if not veh.get("enabled", True):
            print(f"Skipping disabled vehicle: {veh.get('name', 'Unknown')}")
            continue
            
        veh_name = veh.get("name", "Vehicle")
        veh_asset = veh.get("asset", "")
        veh_asset_path = os.path.join(repo_root, veh_asset)
        
        veh_scale = veh.get("scale", [1.0, 1.0, 1.0])
        veh_rot = veh.get("spawn_orientation", veh.get("rotation_euler", [0.0, 0.0, 0.0]))
        veh_pos = veh.get("spawn_position", [0.0, 0.0, 0.0])
        
        prim_path = f"/World/{veh_name}"
        
        if os.path.exists(veh_asset_path):
            strip_embed_physics_scenes(veh_asset_path)
            veh_prim = stage.DefinePrim(prim_path, "Xform")
            
            # Apply transformations
            xformable = UsdGeom.Xformable(veh_prim)
            xformable.AddScaleOp().Set(Gf.Vec3d(*veh_scale))
            xformable.AddRotateXYZOp().Set(Gf.Vec3d(*veh_rot))
            xformable.AddTranslateOp().Set(Gf.Vec3d(*veh_pos))
            
            veh_prim.GetReferences().AddReference(veh_asset_path)
            print(f"Successfully added vehicle '{veh_name}' reference to stage at '{prim_path}'")
        else:
            print(f"Warning: Vehicle asset not found at {veh_asset_path}")

    # Apply PhysicsScene overrides from configuration
    physics_opts = config.get("physics_settings", {})
    if physics_opts:
        from pxr import UsdPhysics, Sdf
        solver_type = physics_opts.get("solver_type", "PGS")
        time_steps = float(physics_opts.get("time_steps_per_second", 120.0))

        scenes_updated = False
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                attr_solver = prim.GetAttribute("physxScene:solverType")
                if attr_solver: attr_solver.Set(solver_type)
                else: prim.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set(solver_type)

                attr_ts = prim.GetAttribute("physxScene:timeStepsPerSecond")
                if attr_ts: attr_ts.Set(time_steps)
                else: prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Float).Set(time_steps)

                if scenes_updated:
                    # A duplicate PhysicsScene was found embedded in a vehicle or environment reference!
                    # This completely breaks the PhysX timeline synchronization and throws overlapping read warnings.
                    # We must natively ERADICATE any extra scenes from the composed USD stage entirely.
                    print(f"[Physics] Warning: Eradicating duplicate PhysicsScene from composed stage at '{prim.GetPath()}'")
                    stage.RemovePrim(prim.GetPath())
                    continue

                print(f"[Physics] Configured Primary PhysicsScene at '{prim.GetPath()}' -> Solver: {solver_type}, Time Steps: {time_steps}")
                scenes_updated = True

        if not scenes_updated:
            print("[Physics] Emitting new singleton PhysicsScene since no assets contained one mapping.")
            scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
            prim = scene.GetPrim()
            prim.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set(solver_type)
            prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Float).Set(time_steps)
            print(f"[Physics] Force-configured Singleton PhysicsScene at '/physicsScene' -> Solver: {solver_type}, Time Steps: {time_steps}")

    # Enable ROS2 and Core nodes with defensive discovery to avoid version conflicts
    import omni.kit.app
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    
    # Pre-scan available extensions
    available_exts = [e.get("id") for e in ext_manager.get_extensions()]
    def enable_preferred(variants):
        found = False
        for v in variants:
            if any(v in ex for ex in available_exts):
                if ext_manager.set_extension_enabled_immediate(v, True):
                    print(f"[Extensions] Enabled compatible variant: {v}")
                    found = True
                    # In some versions like 6.0+, we might need all available variants
                    # for different node types. We continue in those cases.
        return found

    # Enable everything we can for the bridge and core nodes.
    enable_preferred(["isaacsim.ros2.bridge", "omni.isaac.ros2_bridge", 
                      "isaacsim.ros2.nodes", "isaacsim.core_nodes", "omni.isaac.core_nodes"])
    
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    # ── Sensor Topic Remapping ──────────────────────────────────────────────────
    # Remap all hardcoded sensor topics baked into the USD to per-vehicle namespaced topics.
    sensor_topics  = config.get("topics_to_remap", [])
    frame_ids      = config.get("frame_ids_to_remap", [])
    _frame_ids_set = set(frame_ids)

    for veh in vehicles:
        if not veh.get("enabled", True):
            continue
            
        veh_name = veh.get("name", "Vehicle")
        veh_prim_path = f"/World/{veh_name}"
        topic_prefix = veh.get("topic_prefix", f"/{veh_name.lower()}")
        topic_prefix = "/" + topic_prefix.strip("/") # Ensure /ego format
        ackermann_topic = topic_prefix + "/drive"
        
        remapped_count = 0
        veh_prim = stage.GetPrimAtPath(veh_prim_path)
        if not veh_prim.IsValid():
            continue
            
        # Traverse only under this vehicle's prim hierarchy
        for prim in stage.Traverse():
            p_path = prim.GetPath().pathString
            if not p_path.startswith(veh_prim_path):
                continue
            
            # Check for ROS2 Subscription nodes specifically for drive control
            if prim.GetTypeName() == "OmniGraphNode":
                try:
                    node_type = prim.GetAttribute("node:type").Get()
                    if node_type and "SubscribeAckermannDrive" in node_type:
                        topic_attr = prim.GetAttribute("inputs:topicName")
                        if topic_attr:
                            topic_attr.Set(ackermann_topic)
                            print(f"[ROS2 setup] {veh_name}: Drive topic -> '{ackermann_topic}'")
                except Exception: pass

            # ── Sensor Topic Remapping (Exhaustive Scan) ───────────────────────
            # Remap ALL string attributes that match known sensor topics (imu, rgb, tf, etc.)
            for attr in prim.GetAttributes():
                attr_name = attr.GetName().lower()
                # Skip attributes that designate data types or configuration constants
                if "type" in attr_name or "format" in attr_name:
                    continue
                    
                val = attr.Get()
                if isinstance(val, str) and val.strip():
                    normalized = "/" + val.strip("/")
                    if normalized in sensor_topics:
                        new_val = topic_prefix + normalized
                        attr.Set(new_val)
                        remapped_count += 1
                        print(f"[ROS2 remap] {veh_name}: attr '{attr.GetName()}' | '{val}' -> '{new_val}'")
                    elif val in _frame_ids_set and "frameid" in attr_name:
                        new_val = topic_prefix.strip("/") + "/" + val
                        attr.Set(new_val)
                        remapped_count += 1
                        print(f"[ROS2 remap] {veh_name}: frame_id '{attr.GetName()}' | '{val}' -> '{new_val}'")
            
            # Remap ConstantString node_namespace (used by ROS2 nodes as the /tf namespace)
            # These nodes typically have an output 'value' used as a namespace string.
            ns_attr = prim.GetAttribute("inputs:value")
            if ns_attr:
                ns_val = ns_attr.Get()
                if isinstance(ns_val, str) and ns_val.strip("/") in [t.strip("/") for t in sensor_topics]:
                    new_ns = topic_prefix + "/" + ns_val.strip("/")
                    ns_attr.Set(new_ns)
                    remapped_count += 1
                    print(f"[ROS2 remap] {veh_name}: namespace '{ns_val}' -> '{new_ns}'")

            # ── Drone Camera / Follow Path Patching ───────────────────────
            # If the USD contains absolute paths to target prims for follow
            # behaviors, these break after namespacing. Prepend the prefix.
            for attr_name in ("inputs:target_prim", "inputs:targetPrim", "inputs:target"):
                target_attr = prim.GetAttribute(attr_name)
                if target_attr:
                    target_val = target_attr.Get()
                    # OmniGraph target attributes are often strings or Sdf.Path
                    if target_val and str(target_val).startswith("/") and not str(target_val).startswith(veh_prim_path):
                        new_target = f"{veh_prim_path}{str(target_val)}"
                        target_attr.Set(new_target if isinstance(target_val, str) else Sdf.Path(new_target))
                        remapped_count += 1
                        print(f"[Drone fix] {veh_name}: path '{target_val}' -> '{new_target}'")
        
        print(f"[ROS2 remap] {veh_name}: {remapped_count} sensor topic(s) remapped with prefix '{topic_prefix}'")

        # ── Follow Camera Creation is now deferred to the 'BaseLink' discovery phase below ──


    # Force synchronous physics execution to prevent the ROS2 omnigraph from making overlap reads 
    # to rigorous physics variables (like getLinearVelocity) while PhysX is simulating asynchronously.
    
    # Let the extensions and graph initialize properly before acquiring the core context
    simulation_app.update()

    # Isaac Sim Full auto-adds /Environment/defaultLight on startup. Remove it so only
    # the DomeLight baked into the environment USD is active.
    _stage_now = omni.usd.get_context().get_stage()
    for _dl_path in ("/Environment/defaultLight", "/World/Environment/defaultLight"):
        _dl_prim = _stage_now.GetPrimAtPath(_dl_path)
        if _dl_prim.IsValid():
            _stage_now.RemovePrim(_dl_path)
            print(f"[Stage] Removed auto-added defaultLight at {_dl_path}")
    
    # Unconditionally force PhysX into synchronous mode via the underlying C++ carb configuration. 
    # This fundamentally prevents ROS 2 ActionGraph nodes from suffering race collisions evaluating velocity.
    import carb
    carb_settings = carb.settings.get_settings()
    carb_settings.set_bool("/physics/asyncFastSimulation", False)
    carb_settings.set_bool("/physics/updateToUsd", True)
    
    # Enforce strict step-limit alignment to perfectly sync physics ticks with rendering ticks
    app_freq = int(float(config.get("physics_settings", {}).get("time_steps_per_second", 60.0)))
    carb_settings.set_int("/persistent/simulation/minFrameRate", app_freq)
    carb_settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
    carb_settings.set_int("/app/runLoops/main/rateLimitFrequency", app_freq)
    
    # Standard timeline execution loop since omni.isaac.core is deprecated in this build module
    import omni.timeline
    import omni.physx
    
    # Force the physx engine to flush any pending async changes before initiating play
    omni.physx.get_physx_interface().force_load_physics_from_usd()
    
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    
    # Establish Native UI Hardware Teleop Binding utilizing raw Carbon input layers dynamically mapped via ActionGraph (No C-Extension Py3.10 conflicts!)
    import carb.input
    import omni.appwindow
    import omni.graph.core as og
    import omni.timeline
    
    vehicle_teleop_publishers = {}  # Store per-vehicle (ctrl_node, pub_node) metadata
    
    # Identify the appropriate ROS2 publisher node type from the system
    # Modern Isaac Sim uses isaacsim.ros2.bridge, Legacy/Omni versions use omni.isaac.ros2_bridge
    pub_node_type = "isaacsim.ros2.bridge.ROS2PublishAckermannDrive"
    if og.get_node_type(pub_node_type) is None:
        pub_node_type = "omni.isaac.ros2_bridge.ROS2PublishAckermannDrive"
    
    print(f"[Teleop] Using ROS2 Publisher Node Type: {pub_node_type}")

    # Intercept the ActionGraph for EACH vehicle to inject viewport loopback
    # Strategy: Discover the existing AckermannController node and drive it directly PLUS
    #            inject a ROS2 publisher wired to the existing ros2_context for observability.
    for i, veh in enumerate(vehicles):
        veh_name = veh.get("name", f"Vehicle_{i}")
        _veh_prefix = "/" + veh.get("topic_prefix", f"/vehicle_{i}").strip("/")
        veh_topic = _veh_prefix + "/drive"
        if not veh.get("enabled", True): continue
            
        print(f"[Teleop] Processing {veh_name} for Loopback on {veh_topic}...")
        
        ackermann_ctrl_node = None
        ackermann_ctrl_path = ""
        ros2_context_path = ""
        subscribe_node_path = ""

        # 1. Discovery pass: find AckermannController, ros2_context, and subscriber nodes.
        #    Uses OG runtime first; falls back to USD prim attribute so both vehicles are
        #    found even if the second vehicle's graph isn't fully registered at startup.
        for prim in stage.Traverse():
            p_path = prim.GetPath().pathString
            if not p_path.startswith(f"/World/{veh_name}"): continue
            if prim.GetTypeName() != "OmniGraphNode": continue

            # Resolve node type: try OG runtime, then USD prim attribute as fallback
            n_type = ""
            _og_node = None
            try:
                _og_node = og.get_node_by_path(p_path)
                if _og_node and _og_node.is_valid():
                    n_type = _og_node.get_node_type().get_node_type()
            except Exception: pass
            if not n_type:
                try: n_type = prim.GetAttribute("node:type").Get() or ""
                except Exception: pass

            if "AckermannController" in n_type:
                ackermann_ctrl_node = _og_node if (_og_node and _og_node.is_valid()) else og.get_node_by_path(p_path)
                ackermann_ctrl_path = p_path
            elif "ROS2Context" in n_type:
                ros2_context_path = p_path
            elif "SubscribeAckermannDrive" in n_type:
                subscribe_node_path = p_path

        # Find the OnPlaybackTick (or gate) node that drives the subscriber via
        # USD attribute connection traversal.  This is more reliable than OG
        # runtime API because it works even when the second vehicle's OmniGraph
        # is not yet registered in the runtime at startup.
        sub_tick_path = ""
        if subscribe_node_path:
            try:
                _sub_prim = stage.GetPrimAtPath(subscribe_node_path)
                _exec_in_attr = _sub_prim.GetAttribute("inputs:execIn")
                if _exec_in_attr:
                    for _src_path in _exec_in_attr.GetConnections():
                        _src_prim_path = _src_path.GetPrimPath()
                        _src_prim = stage.GetPrimAtPath(_src_prim_path)
                        if _src_prim.IsValid():
                            _src_ntype = _src_prim.GetAttribute("node:type").Get() or ""
                            if any(k in _src_ntype for k in ("OnPlaybackTick", "OnTick", "SimulationGate", "IsaacSimulationGate")):
                                sub_tick_path = str(_src_prim_path)
                                print(f"[Teleop] {veh_name}: subscriber tick node found: {sub_tick_path}")
                                break
            except Exception as _ste:
                print(f"[Teleop] {veh_name}: could not resolve subscriber tick: {_ste}")

        if not ackermann_ctrl_node:
            print(f"[Teleop] WARNING: No AckermannController for {veh_name}. Skipping teleop setup.")
            continue

        try:
            graph_obj = ackermann_ctrl_node.get_graph()
            ackermann_graph_path = graph_obj.get_path_to_graph()
            ctrl_node_name = ackermann_ctrl_path.split("/")[-1]
            
            # Names for nodes we will inject
            tick_node_name = f"TeleopTick_{veh_name}_{i}"
            pub_node_name  = f"ViewportPublisher_{veh_name}_{i}"
            pub_tick_name  = f"PubTick_{veh_name}_{i}"

            # 2. Inject Nodes
            # Try to find a working ReadSimulationTime type
            time_node_type = "omni.isaac.core_nodes.IsaacReadSimulationTime"
            # In some 5.x versions it might be isaacsim.core_nodes.IsaacReadSimulationTime
            # We will try to create the graph nodes and handle failures gracefully
            
            create_cmds = [
                (tick_node_name, "omni.graph.action.OnPlaybackTick"),
                (pub_tick_name, "omni.graph.action.OnPlaybackTick"),
                (pub_node_name, pub_node_type),
            ]
            set_cmds = [
                (f"{pub_node_name}.inputs:topicName", veh_topic),
                (f"{pub_node_name}.inputs:frameId", "base_link"),
            ]
            connect_cmds = [
                (f"{tick_node_name}.outputs:tick", f"{ctrl_node_name}.inputs:execIn"),
                (f"{pub_tick_name}.outputs:tick", f"{pub_node_name}.inputs:execIn"),
            ]
            if ros2_context_path:
                ctx_node_name = ros2_context_path.split("/")[-1]
                connect_cmds.append((f"{ctx_node_name}.outputs:context", f"{pub_node_name}.inputs:context"))

            # 3. Store control metadata FIRST — this must succeed even if node
            #    injection below fails, so the vehicle enters the control loop
            #    and og.Controller.set() can override the subscriber.
            _monitor_topic = veh_topic.rstrip("/") + "_teleop"
            _muted_topic   = veh_topic.rstrip("/") + "_muted"
            # Discover subscriber→controller OmniGraph data connections.
            # These connections carry the last received ROS2 values and always
            # override og.Controller.set() authored values — even when the
            # subscriber's tick is disabled. The only reliable fix is to
            # disconnect them when entering TELEOP and reconnect for ROS2.
            _sub_ctrl_conns = []  # list of (src_attr_path, dst_attr_path)
            if subscribe_node_path and ackermann_ctrl_path:
                _ctrl_prim = stage.GetPrimAtPath(ackermann_ctrl_path)
                for _inp in ("inputs:speed", "inputs:steeringAngle"):
                    _usd_attr = _ctrl_prim.GetAttribute(_inp)
                    if not _usd_attr:
                        continue
                    for _src in _usd_attr.GetConnections():
                        if str(_src.GetPrimPath()) == subscribe_node_path:
                            _sub_ctrl_conns.append((str(_src), f"{ackermann_ctrl_path}.{_inp}"))
            if _sub_ctrl_conns:
                print(f"[Teleop] {veh_name}: found {len(_sub_ctrl_conns)} subscriber→controller connection(s) to manage")

            vehicle_teleop_publishers[veh_name] = {
                "ctrl_attr_speed": f"{ackermann_graph_path}/{ctrl_node_name}.inputs:speed",
                "ctrl_attr_steer": f"{ackermann_graph_path}/{ctrl_node_name}.inputs:steeringAngle",
                "pub_node_path":  f"{ackermann_graph_path}/{pub_node_name}",
                "pub_topic_attr": f"{ackermann_graph_path}/{pub_node_name}.inputs:topicName",
                "drive_topic":    veh_topic,
                "monitor_topic":  _monitor_topic,
                "sub_node_path":  subscribe_node_path,
                "sub_tick_path":  sub_tick_path,
                "sub_topic_attr": f"{subscribe_node_path}.inputs:topicName" if subscribe_node_path else None,
                "muted_topic":    _muted_topic,
                "sub_ctrl_connections": _sub_ctrl_conns,
            }
            print(f"[Teleop] Control paths registered for {veh_name} (graph: {ackermann_graph_path})")

            # 4. Inject observability publisher node — non-critical, failures do
            #    NOT prevent teleop; the vehicle is already in the control loop.
            try:
                og.Controller.edit(graph_obj, {
                    og.Controller.Keys.CREATE_NODES: create_cmds,
                    og.Controller.Keys.SET_VALUES: set_cmds,
                })

                # Optional simulation-time node for accurate message timestamps
                time_node_name = f"ReadSimTime_{veh_name}_{i}"
                try:
                    og.Controller.edit(graph_obj, {
                        og.Controller.Keys.CREATE_NODES: [(time_node_name, time_node_type)],
                    })
                    connect_cmds.append((f"{time_node_name}.outputs:simulationTime", f"{pub_node_name}.inputs:timeStamp"))
                except Exception:
                    try:
                        og.Controller.edit(graph_obj, {
                            og.Controller.Keys.CREATE_NODES: [(time_node_name, "isaacsim.core_nodes.IsaacReadSimulationTime")],
                        })
                        connect_cmds.append((f"{time_node_name}.outputs:simulationTime", f"{pub_node_name}.inputs:timeStamp"))
                    except Exception:
                        print(f"[Teleop] Warning: Could not create IsaacReadSimulationTime for {veh_name}.")

                # Wiring (done separately to handle connection conflicts)
                for src, dst in connect_cmds:
                    try:
                        og.Controller.connect(og.Controller.attribute(f"{ackermann_graph_path}/{src}"),
                                              og.Controller.attribute(f"{ackermann_graph_path}/{dst}"))
                    except Exception:
                        if "execIn" in dst: pass
                        else: print(f"[Teleop] Warning: Could not connect {src} -> {dst}")

                # Route publisher to monitor topic so drive_topic is free for external ROS2
                try:
                    og.Controller.set(
                        og.Controller.attribute(f"{ackermann_graph_path}/{pub_node_name}.inputs:topicName"),
                        _monitor_topic,
                    )
                    print(f"[Teleop] Publisher routed to '{_monitor_topic}' for {veh_name}.")
                except Exception as _rr_e:
                    print(f"[Teleop] Warning: Could not set publisher topic for {veh_name}: {_rr_e}")

            except Exception as _inj_e:
                print(f"[Teleop] Warning: Publisher node injection failed for {veh_name}: {_inj_e}")
                print(f"[Teleop] Control (speed/steer override) still active for {veh_name}.")

        except Exception as e:
            print(f"[Teleop] ERROR during setup for {veh_name}: {e}")

    if not vehicle_teleop_publishers:
        print(f"[Teleop] CRITICAL ERROR: No control nodes registered!")
    else:
        print(f"[Teleop] Successfully initialized {len(vehicle_teleop_publishers)} vehicle control path(s).")

    # ── Sensor Graph Gating ──────────────────────────────────────────────────────
    # For each vehicle, stop disabled sensors from publishing ROS2 data.
    # Strategy: set inputs:enabled = False on the execution-source node
    # (OnPlaybackTick / IsaacSimulationGate) to halt the whole graph, or on
    # the individual camera / lidar publisher nodes for partial disables.
    # USD SetActive() is NOT used — the OmniGraph runtime caches graphs after
    # load and ignores prim activation state changes made at runtime.
    _CAM_NODE_KW  = ("CameraHelper", "PublishImage", "PublishRgb", "RgbSensor")
    _LID_NODE_KW  = ("Lidar", "RTXLidar", "PointCloud", "PublishPointCloud", "LaserScan")
    _TICK_NODE_KW = ("OnPlaybackTick", "OnTick", "SimulationGate", "IsaacSimulationGate")

    def _sg_set_enabled(node_path, enabled):
        """Set inputs:enabled on an OmniGraph node. Returns True if successful."""
        try:
            og.Controller.set(og.Controller.attribute(f"{node_path}.inputs:enabled"), enabled)
            return True
        except Exception:
            return False

    for _sg_veh in vehicles:
        if not _sg_veh.get("enabled", True): continue
        _sg_name   = _sg_veh.get("name", "")
        _sg_en_cam = _sg_veh.get("enable_camera", True)
        _sg_en_lid = _sg_veh.get("enable_lidar",  True)
        if _sg_en_cam and _sg_en_lid:
            continue  # both enabled — nothing to gate

        _sg_base = f"/World/{_sg_name}"
        for _sg_prim in stage.Traverse():
            _sg_pp = _sg_prim.GetPath().pathString
            if not _sg_pp.startswith(_sg_base):
                continue
            if _sg_prim.GetTypeName() != "OmniGraphNode":
                continue
            _sg_node_name = _sg_prim.GetName().lower()
            if not _sg_en_cam and "camera_helper" in _sg_node_name:
                if _sg_set_enabled(_sg_pp, False):
                    print(f"[Sensors] {_sg_name}: Disabled camera helper '{_sg_pp}'")
            elif not _sg_en_lid and "lidar_helper" in _sg_node_name:
                if _sg_set_enabled(_sg_pp, False):
                    print(f"[Sensors] {_sg_name}: Disabled lidar helper '{_sg_pp}'")

    # ── ROS2 Drive Bridge (AckermannDriveStamped + autoware_control_msgs/Control) ──
    # Isaac Sim runs Python 3.12 but rclpy is compiled for Python 3.10 on ROS Humble.
    # Delegate subscriptions to a Python 3.10 subprocess (drive_bridge.py), using the
    # same pattern as gnss_bridge.py.  Commands arrive via subprocess stdout and are
    # stored in _drive_cmds for the physics loop.  Map / TF data is sent to the
    # subprocess via stdin after the scene has warmed up.
    _ros_bridge_enabled = False
    _drive_proc         = None   # subprocess.Popen handle
    _drive_cmds         = {}     # veh_name -> {speed, steer, stamp, source}
    _drive_log          = None
    try:
        import subprocess  as _drv_sub
        import os          as _drv_os
        import time        as _drv_time
        import threading   as _drv_threading

        _drv_script = _drv_os.path.join(
            _drv_os.path.dirname(_drv_os.path.abspath(__file__)),
            "drive_bridge.py")
        _drv_env = {
            "HOME":              _drv_os.environ.get("HOME", "/root"),
            "USER":              _drv_os.environ.get("USER", "root"),
            "PATH":              "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
            "PYTHONPATH":        ("/opt/ros/humble/lib/python3.10/site-packages:"
                                  "/opt/ros/humble/local/lib/python3.10/dist-packages"),
            "AMENT_PREFIX_PATH": "/opt/ros/humble",
            "LD_LIBRARY_PATH":   "/opt/ros/humble/lib",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "ROS_DOMAIN_ID":     _drv_os.environ.get("ROS_DOMAIN_ID", "0"),
        }
        # Do NOT forward ROS_DISCOVERY_SERVER or FASTRTPS_DEFAULT_PROFILES_FILE.
        # drive_bridge uses simple DDS discovery (same as gnss_bridge) so that
        # Autoware and plain ros2 tools can find its subscriptions without needing
        # the discovery server configured.

        _drv_log_path = _drv_os.path.join(
            _drv_os.path.dirname(_drv_os.path.abspath(__file__)),
            "drive_bridge.log")
        _drive_log = open(_drv_log_path, "w")
        _drive_proc = _drv_sub.Popen(
            ["/usr/bin/python3.10", _drv_script],
            stdin=_drv_sub.PIPE,
            stdout=_drv_sub.PIPE,
            stderr=_drive_log,
            text=True,
            env=_drv_env,
        )

        # Send vehicle subscription registrations.
        for _veh in vehicles:
            if not _veh.get("enabled", True): continue
            _vn = _veh.get("name", "Vehicle")
            _vp = "/" + _veh.get("topic_prefix", f"/{_vn.lower()}").strip("/")
            _drive_proc.stdin.write(f"sub\t{_vn}\t{_vp}/drive\t{_vp}/control\n")
        _drive_proc.stdin.write("start\n")
        _drive_proc.stdin.flush()

        # Wait for bridge to signal readiness.
        _drv_ready = _drive_proc.stdout.readline().strip()
        _drive_log.flush()
        if _drv_ready != "ready":
            raise RuntimeError(
                f"drive_bridge.py did not start correctly (got: {_drv_ready!r}); "
                f"see {_drv_log_path} for details")

        # Background thread: read drive commands from subprocess stdout.
        def _drive_reader():
            for _dl in _drive_proc.stdout:
                _dl = _dl.strip()
                if not _dl.startswith("cmd\t"):
                    continue
                try:
                    _, _dvn, _spd, _str, _src = _dl.split("\t")
                    _drive_cmds[_dvn] = {
                        "speed":  float(_spd),
                        "steer":  float(_str),
                        "stamp":  _drv_time.monotonic(),
                        "source": _src,
                    }
                except Exception:
                    pass

        _drv_threading.Thread(target=_drive_reader, daemon=True).start()
        _ros_bridge_enabled = True
        print("[ROS2 Bridge] Drive bridge ready — AckermannDriveStamped on /drive, "
              "autoware_control_msgs/Control on /control")
    except Exception as _bridge_err:
        print(f"[ROS2 Bridge] Could not initialize drive bridge: {_bridge_err}")

    # ── Global Timestamp Fix ───────────────────────────────────────────────────
    # Runs after timeline.play() so the OmniGraph runtime is fully initialized.
    # Uses node.get_attributes() to enumerate safely — calling node.get_attribute()
    # directly on a non-existent attribute triggers C++ plugin errors in the log.
    #
    # The IsaacReadSimulationTime extension may not be loadable via CREATE_NODES
    # with hardcoded type names (extension not registered for dynamic creation),
    # even though baked-in USD instances work at load time.
    # Solution: discover the exact registered type from an existing live instance.
    _sim_time_node_type = None
    for _p in stage.Traverse():
        if _p.GetTypeName() != "OmniGraphNode":
            continue
        try:
            _n = og.get_node_by_path(_p.GetPath().pathString)
            if not _n.is_valid():
                continue
            _ntype = _n.get_node_type().get_node_type()
            if "ReadSimulationTime" in _ntype:
                _sim_time_node_type = _ntype
                print(f"[Timestamp Fix] Discovered sim time node type: {_ntype}")
                break
        except Exception:
            pass
    if _sim_time_node_type is None:
        print("[Timestamp Fix] WARNING: Could not discover IsaacReadSimulationTime type — timestamps may remain 0")

    _graphs_needing_fix = {}  # graph_path_str -> (graph_obj, [node_local.inputs:timeStamp, ...])
    for prim in stage.Traverse():
        if prim.GetTypeName() != "OmniGraphNode":
            continue
        try:
            node = og.get_node_by_path(prim.GetPath().pathString)
            if not node.is_valid():
                continue
            # Enumerate attributes safely to avoid C++ "attribute not found" errors
            attr_names = {a.get_name() for a in node.get_attributes()}
            if "inputs:timeStamp" not in attr_names:
                continue
            ts_attr = node.get_attribute("inputs:timeStamp")
            if ts_attr.get_upstream_connection_count() > 0:
                continue  # already driven
            graph_obj = node.get_graph()
            graph_path = graph_obj.get_path_to_graph()
            node_local = prim.GetPath().pathString.split("/")[-1]
            if graph_path not in _graphs_needing_fix:
                _graphs_needing_fix[graph_path] = (graph_obj, [])
            _graphs_needing_fix[graph_path][1].append(f"{node_local}.inputs:timeStamp")
            print(f"[Timestamp Fix] Found undriven timeStamp on: {prim.GetPath().pathString}")
        except Exception:
            pass

    _ts_fix_node_name = "GlobalSimTimeReader"
    for graph_path, (graph_obj, dst_attrs) in _graphs_needing_fix.items():
        # Step 1: Look for any existing IsaacReadSimulationTime node already in this graph
        # (e.g. ReadSimTime_*_* injected by teleop, or an existing baked-in instance).
        # Prefer this over creating a new one, since dynamic CREATE_NODES can fail on some builds.
        time_src_node = None
        for _p in stage.Traverse():
            if not _p.GetPath().pathString.startswith(graph_path + "/"):
                continue
            if _p.GetTypeName() != "OmniGraphNode":
                continue
            try:
                _n = og.get_node_by_path(_p.GetPath().pathString)
                if _n.is_valid() and "ReadSimulationTime" in _n.get_node_type().get_node_type():
                    time_src_node = _n
                    print(f"[Timestamp Fix] Using existing sim time node: {_p.GetPath().pathString}")
                    break
            except Exception:
                pass

        # Step 2: If no existing node, try to create one (log the error if it fails)
        if time_src_node is None:
            for _type in (_sim_time_node_type, "isaacsim.core_nodes.IsaacReadSimulationTime",
                          "omni.isaac.core_nodes.IsaacReadSimulationTime"):
                try:
                    og.Controller.edit(graph_obj, {
                        og.Controller.Keys.CREATE_NODES: [(_ts_fix_node_name, _type)],
                    })
                    _candidate = og.get_node_by_path(f"{graph_path}/{_ts_fix_node_name}")
                    if _candidate and _candidate.is_valid():
                        time_src_node = _candidate
                        print(f"[Timestamp Fix] Created sim time node ({_type}) in {graph_path}")
                        break
                except Exception as _ce:
                    print(f"[Timestamp Fix] CREATE_NODES({_type}) failed for {graph_path}: {_ce}")

        if time_src_node is None or not time_src_node.is_valid():
            print(f"[Timestamp Fix] No sim time source available for {graph_path} — skipping")
            continue

        # Step 3: Connect via direct node/attribute references (avoids og.Controller.attribute
        # path-string resolution, which fails if the node was not successfully created above)
        src_attr = time_src_node.get_attribute("outputs:simulationTime")
        if not src_attr.is_valid():
            print(f"[Timestamp Fix] outputs:simulationTime not found on time node in {graph_path}")
            continue

        for dst in dst_attrs:
            node_local = dst.split(".")[0]
            try:
                dst_node = og.get_node_by_path(f"{graph_path}/{node_local}")
                if not dst_node.is_valid():
                    continue
                dst_attr = dst_node.get_attribute("inputs:timeStamp")
                if not dst_attr.is_valid():
                    continue
                og.Controller.connect(src_attr, dst_attr)
                print(f"[Timestamp Fix] Connected sim time -> {graph_path}/{dst}")
            except Exception as _e:
                print(f"[Timestamp Fix] Warning: Could not connect -> {graph_path}/{dst}: {_e}")

    # ── OmniGraph Sensor Topic Remapping (Post-Play Pass) ────────────────────────
    # The pre-play USD pass only catches authored attributes (prim.GetAttributes()).
    # OmniGraph nodes whose inputs:topicName was never explicitly set in the USD use
    # default values (e.g. "imu", "rgb") that are invisible to the USD scan.
    # After timeline.play() the OmniGraph runtime is live, so node.get_attributes()
    # exposes all inputs including defaults — this pass catches the missed ones.
    _sensor_topics_set = set(sensor_topics)
    for veh in vehicles:
        if not veh.get("enabled", True):
            continue
        veh_name = veh.get("name", "Vehicle")
        veh_prim_path = f"/World/{veh_name}"
        topic_prefix = "/" + veh.get("topic_prefix", f"/{veh_name.lower()}").strip("/")

        _og_remap_count = 0
        for prim in stage.Traverse():
            p_path = prim.GetPath().pathString
            if not p_path.startswith(veh_prim_path) or prim.GetTypeName() != "OmniGraphNode":
                continue
            try:
                node = og.get_node_by_path(p_path)
                if not node.is_valid():
                    continue
                og_attr_names = {a.get_name() for a in node.get_attributes()}
                for candidate in ("inputs:topicName", "inputs:nodeNamespace"):
                    if candidate not in og_attr_names:
                        continue
                    if "type" in candidate.lower() or "format" in candidate.lower():
                        continue
                    # Try OG attr.get() first, fall back to USD prim attribute
                    val = None
                    og_attr = node.get_attribute(candidate)
                    for _getter in (lambda: og_attr.get(),
                                    lambda: prim.GetAttribute(candidate).Get()):
                        try:
                            v = _getter()
                            if isinstance(v, str) and v.strip():
                                val = v
                                break
                        except Exception:
                            pass
                    if not isinstance(val, str) or not val.strip():
                        continue
                    normalized = "/" + val.strip("/")
                    if normalized not in _sensor_topics_set:
                        continue
                    if val.startswith(topic_prefix):
                        continue  # already remapped by the pre-play pass
                    new_val = topic_prefix + normalized
                    # Write via USD prim API (reliable) and attempt OG attr as well
                    prim.GetAttribute(candidate).Set(new_val)
                    try:
                        og_attr.set(new_val)
                    except Exception:
                        pass
                    _og_remap_count += 1
                    print(f"[ROS2 remap OG] {veh_name}: {prim.GetPath().GetName()}.{candidate} "
                          f"'{val}' -> '{new_val}'")
            except Exception:
                pass
        if _og_remap_count > 0:
            print(f"[ROS2 remap OG] {veh_name}: {_og_remap_count} additional topic(s) remapped post-play")

        # ── Frame ID remapping (post-play OG pass) ────────────────────────────
        # Scans all OmniGraph node attributes whose name contains "FrameId" and
        # remaps base values ("odom", "base_link") to namespaced ones ("ego/odom").
        if _frame_ids_set:
            _pfx_ns = topic_prefix.strip("/")  # e.g. "ego"
            _fid_count = 0
            for prim in stage.Traverse():
                p_path = prim.GetPath().pathString
                if not p_path.startswith(veh_prim_path) or prim.GetTypeName() != "OmniGraphNode":
                    continue
                try:
                    node = og.get_node_by_path(p_path)
                    if not node.is_valid(): continue
                    for _oa in node.get_attributes():
                        _aname = _oa.get_name()
                        if "frameid" not in _aname.lower(): continue
                        try:
                            _fval = _oa.get()
                            if not isinstance(_fval, str) or not _fval.strip(): continue
                            if _fval not in _frame_ids_set: continue
                            if _fval.startswith(_pfx_ns + "/"): continue
                            _new_fval = _pfx_ns + "/" + _fval
                            prim.GetAttribute(_aname).Set(_new_fval)
                            try: _oa.set(_new_fval)
                            except Exception: pass
                            _fid_count += 1
                            print(f"[ROS2 remap OG] {veh_name}: {prim.GetPath().GetName()}.{_aname} "
                                  f"'{_fval}' -> '{_new_fval}'")
                        except Exception: pass
                except Exception: pass
            if _fid_count > 0:
                print(f"[ROS2 remap OG] {veh_name}: {_fid_count} frame ID(s) remapped post-play")

    # ── Post-play sensor helper disable (OG runtime now fully initialized) ────
    for _sg_veh in vehicles:
        if not _sg_veh.get("enabled", True): continue
        _sg_name   = _sg_veh.get("name", "")
        _sg_en_cam = _sg_veh.get("enable_camera", True)
        _sg_en_lid = _sg_veh.get("enable_lidar",  True)
        if _sg_en_cam and _sg_en_lid:
            continue
        _sg_base = f"/World/{_sg_name}"
        for _sg_prim in stage.Traverse():
            _sg_pp = _sg_prim.GetPath().pathString
            if not _sg_pp.startswith(_sg_base):
                continue
            if _sg_prim.GetTypeName() != "OmniGraphNode":
                continue
            _sg_node_name = _sg_prim.GetName().lower()
            if not _sg_en_cam and "camera_helper" in _sg_node_name:
                if _sg_set_enabled(_sg_pp, False):
                    print(f"[Sensors] {_sg_name}: Disabled camera helper '{_sg_pp}' (post-play)")
            elif not _sg_en_lid and "lidar_helper" in _sg_node_name:
                if _sg_set_enabled(_sg_pp, False):
                    print(f"[Sensors] {_sg_name}: Disabled lidar helper '{_sg_pp}' (post-play)")

    # ── Map Generation and Publishing ─────────────────────────────────────────
    # Renders one top-down orthographic semantic segmentation frame to build a
    # nav_msgs/OccupancyGrid from the environment meshes, then publishes it on
    # /map with TRANSIENT_LOCAL (latched) QoS so late subscribers receive it.
    # Also publishes a static map→odom identity transform so the TF tree is
    # complete: map → odom → base_link.
    _map_cfg     = config.get("map_server", {})
    _map_enabled = _map_cfg.get("enabled", False)

    if _map_enabled and _ros_bridge_enabled and _drive_proc:
        try:
            import numpy as _map_np
            import omni.replicator.core as _map_rep
            import struct  as _map_struct
            import base64  as _map_b64
            from pxr import UsdGeom as _MapUG, Gf as _MapGf, Usd as _MapUsd

            _map_res   = float(_map_cfg.get("resolution", 0.05))
            _map_stage = omni.usd.get_context().get_stage()

            # Assign semantic labels to environment meshes if not done already.
            # seg_id_keywords / seg_default_id / seg_enabled are defined later in
            # the segmentation setup block; use locals().get() to handle the case
            # where the map section runs before that block.
            _map_id_kws = locals().get("seg_id_keywords") or {0: ["default"], 1: ["track"]}
            _map_def_id = locals().get("seg_default_id", 0)

            def _map_assign_label(prim, label_str):
                try:
                    from omni.isaac.core.utils.semantics import add_update_semantics
                    add_update_semantics(prim, label_str, type_label="class"); return
                except Exception: pass
                try:
                    from pxr import Semantics as _SA
                    _s = _SA.SemanticsAPI.Apply(prim, "Semantics")
                    _s.CreateSemanticTypeAttr().Set("class")
                    _s.CreateSemanticDataAttr().Set(label_str)
                except Exception: pass

            if not locals().get("seg_enabled", False):
                _lbl_n = 0
                for _mp in _map_stage.Traverse():
                    if _mp.GetTypeName() != "Mesh": continue
                    if not _mp.GetPath().pathString.startswith("/World/Environment"): continue
                    _mn = _mp.GetName().lower()
                    _aid = _map_def_id
                    for _mid, _mkws in _map_id_kws.items():
                        if "default" in _mkws: continue
                        if any(_kw.lower() in _mn for _kw in _mkws):
                            _aid = _mid; break
                    _map_assign_label(_mp, str(_aid))
                    _lbl_n += 1
                if _lbl_n:
                    print(f"[Map] Assigned semantic labels to {_lbl_n} environment meshes")

            # Compute environment bounding box (world space, 5 % margin).
            _map_env = _map_stage.GetPrimAtPath("/World/Environment")
            _map_bbc = _MapUG.BBoxCache(_MapUsd.TimeCode.Default(), ["default", "render"])
            _map_br  = _map_bbc.ComputeWorldBound(_map_env).GetRange()
            _map_mn_pt, _map_mx_pt = _map_br.GetMin(), _map_br.GetMax()
            _map_ex  = (_map_mx_pt[0] - _map_mn_pt[0]) * 1.05   # X extent + margin
            _map_ey  = (_map_mx_pt[1] - _map_mn_pt[1]) * 1.05   # Y extent + margin
            _map_cx  = (_map_mn_pt[0] + _map_mx_pt[0]) / 2
            _map_cy  = (_map_mn_pt[1] + _map_mx_pt[1]) / 2
            _map_cz  = float(_map_mx_pt[2]) + 20.0               # 20 m above highest point
            _map_orig_x = _map_cx - _map_ex / 2
            _map_orig_y = _map_cy - _map_ey / 2
            _map_pw  = min(4096, max(64, int(_map_ex / _map_res)))
            _map_ph  = min(4096, max(64, int(_map_ey / _map_res)))

            # Create top-down orthographic camera.
            # aperture in "tenths of scene units" (= cm when stage unit = m).
            _map_cp  = "/World/_MapCamera"
            _map_cam = _MapUG.Camera.Define(_map_stage, _map_cp)
            _map_cam.GetProjectionAttr().Set(_MapUG.Tokens.orthographic)
            _map_cam.GetHorizontalApertureAttr().Set(_map_ex * 10.0)
            _map_cam.GetVerticalApertureAttr().Set(_map_ey * 10.0)
            _map_cam.GetClippingRangeAttr().Set(
                _MapGf.Vec2f(0.1, _map_cz + abs(float(_map_mn_pt[2])) + 10.0))
            _map_xf = _MapUG.Xformable(_map_cam)
            _map_xf.ClearXformOpOrder()
            # Identity rotation: camera looks in -Z (straight down in Z-up world).
            _map_xf.AddTranslateOp().Set(_MapGf.Vec3d(_map_cx, _map_cy, _map_cz))

            # Attach semantic annotator, warm up renderer, then capture.
            _map_rp     = _map_rep.create.render_product(_map_cp, (_map_pw, _map_ph))
            _map_annot  = _map_rep.AnnotatorRegistry.get_annotator(
                "semantic_segmentation", init_params={"colorize": False})
            _map_annot.attach(_map_rp)
            print(f"[Map] Top-down camera ({_map_cx:.1f},{_map_cy:.1f},{_map_cz:.1f}), "
                  f"coverage {_map_ex:.1f}×{_map_ey:.1f} m → {_map_pw}×{_map_ph} px")
            for _ in range(10):
                simulation_app.update()

            _map_sd  = _map_annot.get_data()
            _map_i2l = _map_sd.get("info", {}).get("idToLabels", {})
            _map_arr = _map_sd.get("data")   # (H, W) uint32

            if _map_arr is not None and _map_arr.size > 0:
                # Map replicator IDs → occupancy values (0=free, 100=occupied, -1=unknown).
                _map_l2v = {}
                for _rid, _li in _map_i2l.items():
                    try:
                        _oc = int(_li.get("class", str(_map_def_id)))
                        _map_l2v[int(_rid)] = 0 if _oc != _map_def_id else 100
                    except (ValueError, TypeError):
                        _map_l2v[int(_rid)] = -1
                _map_occ = _map_np.vectorize(
                    lambda v: _map_l2v.get(int(v), -1), otypes=[_map_np.int8])(_map_arr)
                # Image row 0 = world max_Y; OccupancyGrid row 0 = world min_Y → flip.
                _map_occ = _map_np.flipud(_map_occ)

                # Serialize occupancy data as base64-packed signed bytes and send
                # to drive_bridge.py subprocess for publishing via rclpy.
                _map_flat = _map_occ.flatten().tolist()
                _map_raw  = _map_struct.pack(f"{len(_map_flat)}b", *_map_flat)
                _map_db64 = _map_b64.b64encode(_map_raw).decode("ascii")
                _drive_proc.stdin.write(
                    f"map\t{int(_map_pw)}\t{int(_map_ph)}\t{_map_res}"
                    f"\t{float(_map_orig_x)}\t{float(_map_orig_y)}\t{_map_db64}\n")
                _drive_proc.stdin.flush()
                print(f"[Map] Sent /map to bridge: {_map_pw}×{_map_ph} cells @ {_map_res} m/cell")

                # Static map→{prefix}/odom identity transforms, one per enabled vehicle.
                _odom_frames = []
                for _mv in vehicles:
                    if not _mv.get("enabled", True): continue
                    _mvpfx = "/" + _mv.get("topic_prefix",
                                          f"/{_mv.get('name','').lower()}").strip("/")
                    _mv_odom_frame = _mvpfx.strip("/") + "/odom"
                    _drive_proc.stdin.write(f"tf\tmap\t{_mv_odom_frame}\n")
                    _odom_frames.append(_mv_odom_frame)
                _drive_proc.stdin.flush()
                print(f"[Map] Sent static map→odom TFs to bridge: {', '.join(_odom_frames)}")
            else:
                print("[Map] WARNING: semantic annotator returned no data — /map not published")

            try: _map_rp.destroy()
            except Exception: pass
            try: _map_stage.RemovePrim(_map_cp)
            except Exception: pass

        except Exception as _map_err:
            import traceback as _map_tb
            print(f"[Map] Error during map generation: {_map_err}")
            _map_tb.print_exc()

    # ── GNSS Sensor Setup ─────────────────────────────────────────────────────
    # Reads each vehicle's base_link world position from USD every frame, converts
    # it to WGS-84 coordinates via equirectangular projection from the configured
    # map origin, adds configurable Gaussian noise, and publishes
    # sensor_msgs/NavSatFix on /{topic_prefix}/gnss at the requested rate.
    _gnss_cfg         = config.get("gnss", {})
    _gnss_enabled     = _gnss_cfg.get("enabled", False)
    _gnss_publishers  = {}   # veh_name -> rclpy Publisher<NavSatFix>
    _gnss_bridge      = None  # rclpy node used for publisher creation and clock
    _gnss_frame_skip  = 1
    _gnss_lat0 = _gnss_lon0 = _gnss_alt0 = 0.0
    _gnss_h_std = _gnss_v_std = 0.0
    _gnss_cos_lat0 = 1.0
    _gnss_hud_data = {}  # veh_name -> (lat, lon) for HUD display
    _R_EARTH = 6_371_000.0   # mean Earth radius, metres

    if _gnss_enabled:
        try:
            import math   as _gnss_math
            import random as _gnss_random

            _gnss_origin     = _gnss_cfg.get("map_origin", {})
            _gnss_lat0       = float(_gnss_origin.get("latitude",  0.0))
            _gnss_lon0       = float(_gnss_origin.get("longitude", 0.0))
            _gnss_alt0       = float(_gnss_origin.get("altitude",  0.0))
            _gnss_noise      = _gnss_cfg.get("noise", {})
            _gnss_h_std      = float(_gnss_noise.get("horizontal_stddev_m", 0.5))
            _gnss_v_std      = float(_gnss_noise.get("altitude_stddev_m",   1.0))
            _gnss_rate       = float(_gnss_cfg.get("publish_rate_hz", 10.0))
            _gnss_frame_skip = max(1, int(round(app_freq / _gnss_rate)))
            _gnss_cos_lat0   = _gnss_math.cos(_gnss_math.radians(_gnss_lat0))

            # Isaac Sim's Python 3.12 cannot import rclpy (compiled for 3.10).
            # Delegate publishing to a Python 3.10 subprocess (gnss_bridge.py)
            # that reads tab-separated GNSS records from stdin and publishes
            # sensor_msgs/NavSatFix via rclpy.
            import subprocess as _gnss_subprocess
            import os as _gnss_os
            import time as _gnss_time

            _gnss_script = _gnss_os.path.join(
                _gnss_os.path.dirname(_gnss_os.path.abspath(__file__)),
                "gnss_bridge.py")
            # Use a minimal environment to avoid Isaac Sim's LD_LIBRARY_PATH
            # and PYTHONPATH contaminating the Python 3.10 subprocess.
            _gnss_env = {
                "HOME":             _gnss_os.environ.get("HOME", "/root"),
                "USER":             _gnss_os.environ.get("USER", "root"),
                "PATH":             "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
                "PYTHONPATH":       ("/opt/ros/humble/lib/python3.10/site-packages:"
                                     "/opt/ros/humble/local/lib/python3.10/dist-packages"),
                "AMENT_PREFIX_PATH": "/opt/ros/humble",
                "LD_LIBRARY_PATH":   "/opt/ros/humble/lib",
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                "ROS_DOMAIN_ID":     _gnss_os.environ.get("ROS_DOMAIN_ID", "0"),
            }

            _gnss_log_path = _gnss_os.path.join(
                _gnss_os.path.dirname(_gnss_os.path.abspath(__file__)),
                "gnss_bridge.log")
            _gnss_log = open(_gnss_log_path, "w")
            _gnss_proc = _gnss_subprocess.Popen(
                ["/usr/bin/python3.10", _gnss_script],
                stdin=_gnss_subprocess.PIPE,
                stdout=_gnss_subprocess.PIPE,
                stderr=_gnss_log,
                text=True,
                env=_gnss_env,
            )
            # Block until the bridge signals it is ready (or dies trying).
            _gnss_ready = _gnss_proc.stdout.readline().strip()
            _gnss_log.flush()
            if _gnss_ready != "ready":
                raise RuntimeError(
                    f"gnss_bridge.py did not start correctly (got: {_gnss_ready!r}); "
                    f"see {_gnss_log_path} for details")
            _gnss_bridge = _gnss_proc

            for _gv in vehicles:
                if not _gv.get("enabled", True):     continue
                if not _gv.get("enable_gnss", True): continue
                _gvname = _gv.get("name", "")
                _gvpfx  = "/" + _gv.get("topic_prefix", f"/{_gvname.lower()}").strip("/")
                _gtopic = _gvpfx + "/gnss"
                _gfid   = _gvpfx.strip("/") + "/base_link"

                _gnss_publishers[_gvname] = {"topic": _gtopic, "frame_id": _gfid}
                print(f"[GNSS] {_gvname}: NavSatFix publisher on '{_gtopic}' "
                      f"at {_gnss_rate:.0f} Hz (every {_gnss_frame_skip} frames)")

            print(f"[GNSS] Map origin: lat={_gnss_lat0:.6f}°  lon={_gnss_lon0:.6f}°  "
                  f"alt={_gnss_alt0:.1f} m  |  noise H={_gnss_h_std} m  V={_gnss_v_std} m")
        except Exception as _gnss_init_err:
            import traceback as _gnss_tb
            print(f"[GNSS] Setup error: {_gnss_init_err}")
            _gnss_tb.print_exc()
            _gnss_enabled = False
    else:
        print("[GNSS] Disabled via config.")

    # ── Discover IMU and Odometry OmniGraph nodes for HUD readback ────────────
    vehicle_sensor_nodes = {}  # veh_name -> {"imu_path": str|None, "odom_path": str|None}
    for veh in vehicles:
        if not veh.get("enabled", True): continue
        veh_name = veh.get("name", "Vehicle")
        veh_prim_path = f"/World/{veh_name}"
        imu_path = None
        odom_path = None

        for prim in stage.Traverse():
            p_path = prim.GetPath().pathString
            if not p_path.startswith(veh_prim_path) or prim.GetTypeName() != "OmniGraphNode":
                continue
            try:
                node = og.get_node_by_path(p_path)
                if not node.is_valid(): continue
                n_type = node.get_node_type().get_node_type()
                if any(x in n_type for x in ["IsaacImuSensor", "ReadIMU"]):
                    imu_path = p_path
                elif any(x in n_type for x in ["ComputeOdometry", "IsaacComputeOdometry"]):
                    odom_path = p_path
            except Exception:
                continue
        vehicle_sensor_nodes[veh_name] = {"imu_path": imu_path, "odom_path": odom_path}
        print(f"[HUD] {veh_name}: odom_path={odom_path}, imu_path={imu_path}")
    
    # ── Follow Camera Persistence Configs ────────────────────────────────
    veh_follow_configs = {} # veh_name -> {base_path, front_path, rear_path, dist, height}
    fc_cfg = config.get("follow_camera", {})
    for veh in vehicles:
        if not veh.get("enabled", True): continue
        # Follow camera is now always enabled for any active vehicle
        
        veh_name = veh.get("name", "Vehicle")
        veh_prim_path = f"/World/{veh_name}"
        
        # Discover chassis and markers once at startup
        # We look for "Chassis" or "base_link" (case-insensitive)
        chassis_prim = None
        front_p = None
        rear_p = None
        
        for p in stage.Traverse():
            path_str = p.GetPath().pathString
            if not path_str.startswith(veh_prim_path): continue
            
            p_name = p.GetName().lower()
            if p.GetTypeName() != "OmniGraphNode":
                # Priority: "chassis" then "base_link"
                if "chassis" in p_name and not chassis_prim:
                    chassis_prim = p
                elif "base_link" in p_name and not chassis_prim:
                    chassis_prim = p
                
                # Markers for forward axis
                if "front" in p_name and not front_p: front_p = p
                if ("rear" in p_name or "back" in p_name) and not rear_p: rear_p = p
            
        if chassis_prim and chassis_prim.IsValid():
            _cam_name_map = {
                "Ego_Vehicle": "EgoFollowCamera",
                "Opponent_Vehicle": "OpponentFollowCamera",
            }
            veh_follow_configs[veh_name] = {
                "base_path": chassis_prim.GetPath().pathString,
                "front_path": front_p.GetPath().pathString if front_p else None,
                "rear_path": rear_p.GetPath().pathString if rear_p else None,
                "dist": float(fc_cfg.get("distance", 5.0)),
                "height": float(fc_cfg.get("height", 2.0)),
                "focus_height": float(fc_cfg.get("focus_height", 0.5)),
                "cam_name": _cam_name_map.get(veh_name, "FollowCamera"),
            }

    follow_cam_handles = {} # Internal tracking for the simulation loop

    # ── Viewport HUD Overlay ──────────────────────────────────────────────────
    try:
        import omni.ui as ui
        
        HUD_ENABLED = True
        
        # ── Color constants (omni.ui = 0xAABBGGRR) ───────────────────────────
        C_BLACK  = 0xFF000000   # fully opaque black
        C_CYAN   = 0xFFFFD400   # bright cyan  (R=0x00 G=0xD4 B=0xFF A=0xFF)
        C_WHITE  = 0xFFFFFFFF   # white
        C_GREY   = 0xFF999999   # muted grey
        C_ORANGE = 0xFF4488FF   # orange       (R=0xFF G=0x88 B=0x44 A=0xFF)
        C_BLUE   = 0xFFFF8800   # dodger blue  (R=0x00 G=0x88 B=0xFF A=0xFF)
        
        CARD_W = 190
        CARD_H = 430
        GAP    = 16

        enabled_vehicles = [v for v in vehicles if v.get("enabled", True)]
        _n_veh = max(1, len(enabled_vehicles))

        # Scale card height so the combined HUD never exceeds the Isaac Sim window height.
        # position_y=100 consumes 100 px at the top; reserve 20 px at the bottom.
        _app_win    = omni.appwindow.get_default_app_window()
        _win_h      = _app_win.get_height() if _app_win else render_res[1]
        _avail_h    = _win_h - 120
        _card_h_fit = int((_avail_h - 16) / _n_veh - GAP)
        CARD_H = int(min(CARD_H, max(100, _card_h_fit)) * 0.7)
        _sc = CARD_H / 430  # scale factor relative to default card height

        CARD_W      = max(120, int(190 * _sc))
        FONT_HEADER = max(9,   int(22  * _sc))
        FONT_ROW    = max(8,   int(18  * _sc))
        FONT_CTRL   = max(6,   int(11  * _sc))
        CARD_MARGIN = max(4,   int(10  * _sc))
        INDENT_W    = max(4,   int(20  * _sc))
        LABEL_W     = max(40,  int(75  * _sc))
        SPACER_SM   = max(1,   int(4   * _sc))

        total_w = CARD_W + 16
        total_h = (CARD_H + GAP) * _n_veh + 16

        hud_window = ui.Window(
            "VehicleStatusHUD",
            width=total_w,
            height=total_h,
            position_x=70,
            position_y=70,
            flags=(
                ui.WINDOW_FLAGS_NO_TITLE_BAR
                | ui.WINDOW_FLAGS_NO_SCROLLBAR
                | ui.WINDOW_FLAGS_NO_RESIZE
                | ui.WINDOW_FLAGS_NO_MOVE
            ),
        )
        # Transparent window frame — cards have their own background
        hud_window.frame.set_style({"background_color": 0x00000000})

        hud_labels = {}

        def _label(text, color, size, w=0, bold=False):
            style = {"color": color, "font_size": size}
            if bold: style["font_style"] = "Bold"
            lbl = ui.Label(text, style=style, width=w if w > 0 else ui.Fraction(1),
                           alignment=ui.Alignment.LEFT_CENTER)
            return lbl

        with hud_window.frame:
            with ui.VStack(spacing=GAP, width=total_w):
                # Sort to ensure Ego is on top
                sorted_vehs = sorted(enabled_vehicles, key=lambda x: "Ego" not in x["name"])
                for veh in sorted_vehs:
                    veh_name = veh.get("name", "Vehicle")
                    models = {}
                    with ui.ZStack(width=CARD_W, height=CARD_H):
                        # Background — semi-transparent dark gray (0xAABBGGRR: A=0x55, RGB=0x383838)
                        ui.Rectangle(style={"background_color": 0x55383838, "border_radius": 8})
                        with ui.VStack(spacing=2, margin=CARD_MARGIN):
                            # Header
                            is_ego = "Ego" in veh_name
                            h_color = C_CYAN if is_ego else C_WHITE
                            header_label = ui.Label(veh_name.replace("_", " "),
                                                    alignment=ui.Alignment.CENTER,
                                                    style={"color": h_color, "font_size": FONT_HEADER, "font_style": "Bold"})
                            models["header"] = header_label

                            def _row(label_text):
                                with ui.HStack():
                                    ui.Spacer(width=INDENT_W)
                                    ui.Label(f"{label_text}:", style={"color": C_WHITE, "font_size": FONT_ROW}, width=LABEL_W)
                                    val_label = ui.Label("--", style={"color": C_WHITE, "font_size": FONT_ROW}, width=ui.Fraction(1))
                                    return val_label

                            ui.Spacer(height=SPACER_SM)
                            ctrl_src_label = _label("CONTROL: KEYBOARD", C_WHITE, FONT_ROW, bold=True)
                            models["ctrl_src"] = ctrl_src_label
                            models["speed"] = _row("Speed")
                            models["steer"] = _row("Steer")

                            ui.Spacer(height=SPACER_SM)
                            _label("ODOMETRY", C_WHITE, FONT_ROW, bold=True)
                            models["pos_x"] = _row("Pos X")
                            models["pos_y"] = _row("Pos Y")
                            models["pos_z"] = _row("Pos Z")
                            models["lin_vel"] = _row("Lin Vel")
                            models["ang_vel"] = _row("Ang Vel")

                            ui.Spacer(height=SPACER_SM)
                            _label("IMU", C_WHITE, FONT_ROW, bold=True)
                            models["lin_acc"] = _row("Lin Acc")
                            models["ang_acc"] = _row("Ang Acc")

                            ui.Spacer(height=SPACER_SM)
                            _label("GNSS", C_WHITE, FONT_ROW, bold=True)
                            models["gnss_lat"] = _row("Lat")
                            models["gnss_lon"] = _row("Lon")

                    hud_labels[veh_name] = models

        print("[HUD] Viewport overlay window created.")
    except Exception as e:
        HUD_ENABLED = False
        hud_labels = {}
        print(f"[HUD] Initialization Error: {e}")

    # Defensive Keyboard Constant Discovery
    def get_key(cand_list):
        for c in cand_list:
            if hasattr(carb.input.KeyboardInput, c):
                return getattr(carb.input.KeyboardInput, c)
        return None

    K_1         = get_key(["ONE", "_1", "KEY_1", "DIGIT_1"])
    K_2         = get_key(["TWO", "_2", "KEY_2", "DIGIT_2"])
    K_F1        = get_key(["F1"])
    K_F2        = get_key(["F2"])
    K_SLASH     = get_key(["SLASH", "KEY_SLASH", "FORWARD_SLASH"])
    K_R         = get_key(["R", "KEY_R"])
    K_BACKSPACE = get_key(["BACKSPACE", "BACK_SPACE", "DELETE", "BS"])
    K_GRAVE     = get_key(["GRAVE", "BACK_QUOTE", "BACKQUOTE", "TILDE", "ACCENT_GRAVE",
                            "GRAVE_ACCENT", "OEM_3", "SECTION"])
    if K_GRAVE is None:
        # Dynamic fallback: scan every KeyboardInput attribute for a grave/backtick match
        for _kname in dir(carb.input.KeyboardInput):
            if any(s in _kname.upper() for s in ("GRAVE", "BACKTICK", "BACK_QUOTE", "TILDE")):
                K_GRAVE = getattr(carb.input.KeyboardInput, _kname)
                print(f"[Input] Found grave/backtick key via scan: {_kname}")
                break

    # Log all discovered key bindings at startup
    print(f"[Input] Key bindings: 1={K_1}, 2={K_2}, F1={K_F1}, F2={K_F2}, /={K_SLASH}, R={K_R}, Backspace={K_BACKSPACE}, `={K_GRAVE}")
    if K_1 is None: print("[Input] Warning: Could not find key for '1'.")
    if K_2 is None: print("[Input] Warning: Could not find key for '2'.")
    if K_F1 is None: print("[Input] Warning: Could not find key for F1.")
    if K_F2 is None: print("[Input] Warning: Could not find key for F2.")
    if K_GRAVE is None: print("[Input] Warning: Could not find backtick/grave key — dumping all KeyboardInput names:")
    if K_GRAVE is None:
        print("  " + ", ".join(n for n in dir(carb.input.KeyboardInput) if not n.startswith("_")))

    # ── Simulation Selection & Interaction State ──────────────────────────────
    selected_vehicle_name = "Ego_Vehicle"
    slash_pressed_last = False
    r_pressed_last = False
    grave_pressed_last = False
    backspace_pressed_last = False
    choice_keys_pressed_last = {}
    if K_1: choice_keys_pressed_last[K_1] = False
    if K_2: choice_keys_pressed_last[K_2] = False
    key1_hold_start = None   # time.monotonic() when key 1 was first pressed
    key2_hold_start = None   # time.monotonic() when key 2 was first pressed
    key1_toggle_fired = False  # True once hold-toggle fires, suppress until release
    key2_toggle_fired = False

    CTRL_HOLD_DURATION = 1.0  # seconds hold required to toggle control mode

    # Per-vehicle explicit control mode: "KEYBOARD_CONTROL" or "ROS2_CONTROL". Toggled by holding 1/2.
    veh_ctrl_mode = {veh["name"]: "KEYBOARD_CONTROL" for veh in vehicles if veh.get("enabled", True)}
    # Track the last mode each vehicle's OmniGraph publisher was routed for,
    # so we only call og.Controller.set() on the topicName when it actually changes.
    _pub_routed_mode = {}  # veh_name -> "KEYBOARD_CONTROL" or "ROS2_CONTROL"

    is_recording = False
    seg_capture_index = 0

    def _set_viewport_camera(path):
        try:
            from omni.kit.viewport.utility import get_active_viewport
            vp = get_active_viewport()
            if vp: vp.camera_path = path
        except Exception: pass

    def switch_selection(new_name):
        nonlocal selected_vehicle_name
        # Always update the follow camera, even if this vehicle is already selected
        if new_name in veh_follow_configs:
            _cfg = veh_follow_configs[new_name]
            _set_viewport_camera(f"/World/{_cfg['cam_name']}")

        # Only update selection state and HUD if vehicle actually changed
        if new_name == selected_vehicle_name: return
        if new_name not in vehicle_teleop_publishers: return

        print(f"[Selection] Switching to {new_name}")
        selected_vehicle_name = new_name

        for v_name, models in hud_labels.items():
            if "header" in models:
                color = C_BLUE if (new_name == v_name) else C_WHITE
                models["header"].style = {"color": color, "font_size": 18, "font_style": "Bold", "alignment": ui.Alignment.CENTER}

    # ── Segmentation Dataset Setup ────────────────────────────────────────────
    seg_cfg = config.get("semantic_segmentation", {})
    seg_enabled = bool(seg_cfg)
    seg_rgb_annot = None
    seg_mask_annot = None
    seg_id_keywords = {}
    seg_color_map = {}
    seg_images_dir = ""
    seg_gt_masks_dir = ""
    seg_overwrite = True
    seg_capture_freq = 1
    seg_default_id = 0

    if seg_enabled:
        try:
            import numpy as _np
            from PIL import Image as _PILImage
            import omni.replicator.core as _rep
            from pxr import Sdf as _Sdf

            seg_id_keywords  = {int(k): v for k, v in seg_cfg.get("id_keywords", {}).items()}
            seg_color_map    = {int(k): tuple(v) for k, v in seg_cfg.get("color_map", {}).items()}
            # capture_frequency is in Hz; convert to a frame interval
            seg_capture_freq = max(1, int(app_freq / max(1, float(seg_cfg.get("capture_frequency", 1)))))
            _seg_res         = seg_cfg.get("image_resolution", [1280, 720])
            seg_images_dir   = os.path.join(repo_root, seg_cfg.get("images_dir", "data/segmentation/images"))
            seg_gt_masks_dir = os.path.join(repo_root, seg_cfg.get("gt_masks_dir", "data/segmentation/masks"))
            seg_overwrite    = bool(seg_cfg.get("overwrite_existing", True))

            _old_umask = os.umask(0)
            try:
                os.makedirs(seg_images_dir,   mode=0o777, exist_ok=True)
                os.makedirs(seg_gt_masks_dir, mode=0o777, exist_ok=True)
            finally:
                os.umask(_old_umask)
            # chmod every intermediate dir from the leaf up to (not including) repo_root
            for _leaf in [seg_images_dir, seg_gt_masks_dir]:
                _d = _leaf
                while _d and _d != repo_root:
                    try:
                        os.chmod(_d, 0o777)
                    except Exception:
                        pass
                    _parent = os.path.dirname(_d)
                    if _parent == _d:
                        break
                    _d = _parent

            # Identify the default (catch-all) label ID
            for _id, _kws in seg_id_keywords.items():
                if "default" in _kws:
                    seg_default_id = _id
                    break

            # Assign semantic labels to Mesh prims only (skipping joints, physics prims,
            # OmniGraph nodes, etc. to avoid corrupting non-visual stage elements).
            def _assign_semantic_label(prim, label_str):
                try:
                    from omni.isaac.core.utils.semantics import add_update_semantics
                    add_update_semantics(prim, label_str, type_label="class")
                    return
                except Exception:
                    pass
                try:
                    from pxr import Semantics as _SemAPI
                    _sem = _SemAPI.SemanticsAPI.Apply(prim, "Semantics")
                    _sem.CreateSemanticTypeAttr().Set("class")
                    _sem.CreateSemanticDataAttr().Set(label_str)
                except Exception:
                    pass

            _seg_stage = omni.usd.get_context().get_stage()
            _labeled = 0
            for _prim in _seg_stage.Traverse():
                if not _prim.IsValid() or _prim.GetTypeName() != "Mesh":
                    continue
                # Only label environment prims — never touch vehicle prims to avoid
                # corrupting vehicle mesh visibility / render state.
                if not _prim.GetPath().pathString.startswith("/World/Environment"):
                    continue
                _name_lower = _prim.GetName().lower()
                _assigned   = seg_default_id
                for _id, _kws in seg_id_keywords.items():
                    if "default" in _kws:
                        continue
                    for _kw in _kws:
                        if _kw.lower() in _name_lower:
                            _assigned = _id
                            break
                    if _assigned != seg_default_id:
                        break
                _assign_semantic_label(_prim, str(_assigned))
                _labeled += 1
            print(f"[Segmentation] Assigned semantic labels to {_labeled} Mesh prims.")

            # Find the RGB camera under /World/Ego_Vehicle.
            # Prefer cameras whose name contains "color" or "rgb".
            # Explicitly skip depth cameras (name contains "depth").
            _ego_cam_path = None
            _ego_cam_fallback = None
            for _prim in _seg_stage.Traverse():
                _ps = _prim.GetPath().pathString
                if not _ps.startswith("/World/Ego_Vehicle") or not _prim.IsA(UsdGeom.Camera):
                    continue
                _cam_name_lower = _prim.GetName().lower()
                if "depth" in _cam_name_lower:
                    continue  # skip depth cameras
                if "color" in _cam_name_lower or "rgb" in _cam_name_lower:
                    _ego_cam_path = _ps
                    break  # best match
                if _ego_cam_fallback is None:
                    _ego_cam_fallback = _ps  # any non-depth camera as fallback
            if _ego_cam_path is None:
                _ego_cam_path = _ego_cam_fallback

            if _ego_cam_path:
                _seg_rp = _rep.create.render_product(_ego_cam_path, tuple(_seg_res))
                seg_rgb_annot  = _rep.AnnotatorRegistry.get_annotator("rgb")
                seg_mask_annot = _rep.AnnotatorRegistry.get_annotator(
                    "semantic_segmentation",
                    init_params={"colorize": False},
                )
                seg_rgb_annot.attach(_seg_rp)
                seg_mask_annot.attach(_seg_rp)
                print(f"[Segmentation] Camera: {_ego_cam_path} | Resolution: {_seg_res[0]}x{_seg_res[1]}")
                print(f"[Segmentation] images_dir : {seg_images_dir}")
                print(f"[Segmentation] gt_masks_dir: {seg_gt_masks_dir}")
                print(f"[Segmentation] Capture: {seg_cfg.get('capture_frequency', 1)} Hz "
                      f"→ every {seg_capture_freq} frames at {app_freq} Hz loop")
                print(f"[Segmentation] Press R to start/stop recording.")
            else:
                print("[Segmentation] WARNING: No UsdGeom.Camera found under /World/Ego_Vehicle — disabled.")
                seg_enabled = False

        except Exception as _seg_err:
            print(f"[Segmentation] Setup error: {_seg_err}")
            seg_enabled = False

    # ── Simulation Loop ───────────────────────────────────────────────────────
    ts = config.get("keyboard_control_settings", {})
    MAX_SPEED = float(ts.get("max_speed_m_s", 15.0))
    MAX_STEER = float(ts.get("max_steer_rad", 0.52))
    ACCEL = float(ts.get("acceleration_m_s2", 5.0))
    DECEL = float(ts.get("deceleration_m_s2", ACCEL * 2.0))
    STEER_SPEED = float(ts.get("steering_speed_rad_s", 1.5))
    
    current_speed = 0.0
    current_steer = 0.0
    dt_teleop = 1.0 / float(app_freq)
    
    input_iface = carb.input.acquire_input_interface()
    keyboard = None
    iteration = 0
    _restart_pending = False  # True for one frame while stop→play transition settles

    print(f"\n[Simulator] Entering Main Loop ({app_freq}Hz)...")
    while simulation_app.is_running():
        simulation_app.update()
        iteration += 1

        # Deferred restart: play is called the frame *after* stop so that PhysX tensor
        # views (odometry / IMU getVelocities) have fully torn down before OmniGraph
        # nodes execute again.  Skip all OmniGraph reads this frame.
        if _restart_pending:
            timeline.play()
            _restart_pending = False
            print("[Sim] Simulation restarted.")
            continue
        
        if not keyboard:
            appwindow = omni.appwindow.get_default_app_window()
            if appwindow: keyboard = appwindow.get_keyboard()
        
        val_w = val_s = val_a = val_d = val_up = val_down = val_left = val_right = val_space = 0.0
        val_1 = val_2 = val_slash = val_f1 = val_f2 = 0.0

        if keyboard:
            val_w = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.W)
            val_s = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.S)
            val_a = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.A)
            val_d = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.D)
            val_up = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.UP)
            val_down = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.DOWN)
            val_left = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.LEFT)
            val_right = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.RIGHT)
            val_space = input_iface.get_keyboard_value(keyboard, carb.input.KeyboardInput.SPACE)
            val_1         = input_iface.get_keyboard_value(keyboard, K_1)         if K_1         else 0.0
            val_2         = input_iface.get_keyboard_value(keyboard, K_2)         if K_2         else 0.0
            val_f1        = input_iface.get_keyboard_value(keyboard, K_F1)        if K_F1        else 0.0
            val_f2        = input_iface.get_keyboard_value(keyboard, K_F2)        if K_F2        else 0.0
            val_slash     = input_iface.get_keyboard_value(keyboard, K_SLASH)     if K_SLASH     else 0.0
            val_r         = input_iface.get_keyboard_value(keyboard, K_R)         if K_R         else 0.0
            val_backspace = input_iface.get_keyboard_value(keyboard, K_BACKSPACE) if K_BACKSPACE else 0.0
            val_grave     = input_iface.get_keyboard_value(keyboard, K_GRAVE)     if K_GRAVE     else 0.0

            # ` — switch to Perspective camera
            if val_grave > 0.1 and not grave_pressed_last:
                _set_viewport_camera("/OmniverseKit_Persp")
                print("[Camera] Switched to Perspective")
            grave_pressed_last = (val_grave > 0.1)

            # Backspace — restart simulation (stop now, play deferred one frame)
            if val_backspace > 0.1 and not backspace_pressed_last:
                print("[Sim] Restarting simulation...")
                timeline.stop()
                current_speed = 0.0
                current_steer = 0.0
                _restart_pending = True
            backspace_pressed_last = (val_backspace > 0.1)

            # 1 / 2 — short press: select vehicle; hold >1.5 s: toggle control mode
            _now = time.monotonic()

            if K_1:
                key1_down = (val_1 > 0.1)
                key1_was_down = choice_keys_pressed_last.get(K_1, False)
                if key1_down:
                    if key1_hold_start is None:
                        key1_hold_start = _now
                        key1_toggle_fired = False
                    elif not key1_toggle_fired and (_now - key1_hold_start) >= CTRL_HOLD_DURATION:
                        _new = "ROS2_CONTROL" if veh_ctrl_mode.get("Ego_Vehicle") == "KEYBOARD_CONTROL" else "KEYBOARD_CONTROL"
                        veh_ctrl_mode["Ego_Vehicle"] = _new
                        print(f"\n[Control] Ego_Vehicle → {_new}")
                        key1_toggle_fired = True
                else:
                    if key1_was_down and not key1_toggle_fired:
                        # released before hold threshold — switch camera on release
                        switch_selection("Ego_Vehicle")
                    key1_hold_start = None
                    key1_toggle_fired = False
                choice_keys_pressed_last[K_1] = key1_down

            if K_2:
                key2_down = (val_2 > 0.1)
                key2_was_down = choice_keys_pressed_last.get(K_2, False)
                if key2_down:
                    if key2_hold_start is None:
                        key2_hold_start = _now
                        key2_toggle_fired = False
                    elif not key2_toggle_fired and (_now - key2_hold_start) >= CTRL_HOLD_DURATION:
                        _new = "ROS2_CONTROL" if veh_ctrl_mode.get("Opponent_Vehicle") == "KEYBOARD_CONTROL" else "KEYBOARD_CONTROL"
                        veh_ctrl_mode["Opponent_Vehicle"] = _new
                        print(f"\n[Control] Opponent_Vehicle → {_new}")
                        key2_toggle_fired = True
                else:
                    if key2_was_down and not key2_toggle_fired:
                        # released before hold threshold — switch camera on release
                        switch_selection("Opponent_Vehicle")
                    key2_hold_start = None
                    key2_toggle_fired = False
                choice_keys_pressed_last[K_2] = key2_down

            # HUD Toggle
            if val_slash > 0.1 and not slash_pressed_last:
                hud_window.visible = not hud_window.visible
            slash_pressed_last = (val_slash > 0.1)

            # R — toggle segmentation recording
            if val_r > 0.1 and not r_pressed_last:
                is_recording = not is_recording
                if is_recording:
                    seg_capture_index = 0
                    print("\n[Record] Recording STARTED — saving segmentation data.")
                    print(f"[Record] seg_enabled={seg_enabled} | "
                          f"rgb_annot={seg_rgb_annot is not None} | "
                          f"mask_annot={seg_mask_annot is not None}")
                else:
                    print(f"\n[Record] Recording STOPPED — {seg_capture_index} frame(s) saved.")
            r_pressed_last = (val_r > 0.1)
        
        # Teleop Logic
        pressed_throttle = (val_w > 0.0 or val_up > 0.0 or val_s > 0.0 or val_down > 0.0 or val_space > 0.0)
        pressed_steer = (val_a > 0.0 or val_left > 0.0 or val_d > 0.0 or val_right > 0.0 or val_space > 0.0)
        f_dt = float(dt_teleop)
        if val_w > 0.0 or val_up > 0.0:    current_speed += ACCEL * f_dt
        elif val_s > 0.0 or val_down > 0.0: current_speed -= ACCEL * f_dt
        if val_a > 0.0 or val_left > 0.0:  current_steer += STEER_SPEED * f_dt
        elif val_d > 0.0 or val_right > 0.0: current_steer -= STEER_SPEED * f_dt
        if val_space > 0.0: current_speed = current_steer = 0.0

        # Autocentering
        if not pressed_throttle:
            if current_speed > 0.0: current_speed = max(0.0, current_speed - DECEL * f_dt)
            elif current_speed < 0.0: current_speed = min(0.0, current_speed + DECEL * f_dt)
        if not pressed_steer:
            if current_steer > 0.0: current_steer = max(0.0, current_steer - (STEER_SPEED * 2.0) * f_dt)
            elif current_steer < 0.0: current_steer = min(0.0, current_steer + (STEER_SPEED * 2.0) * f_dt)
        current_speed = max(min(current_speed, MAX_SPEED), -MAX_SPEED)
        current_steer = max(min(current_steer, MAX_STEER), -MAX_STEER)
        
        # Drive commands arrive via _drive_reader background thread (drive_bridge.py subprocess).

        # Apply Control — each vehicle has an explicit mode set by F1/F2 toggle.
        # TELEOP: keyboard drives the selected vehicle; non-selected vehicle is idle.
        # ROS2:   external ROS 2 command drives the vehicle; keyboard is ignored.
        #         Display reads back the OmniGraph controller state so it reflects
        #         whatever the vehicle's built-in ROS2 subscriber applied, not just
        #         what our rclpy bridge received.
        def _read_og_ctrl(meta):
            """Read speed/steer directly from the OmniGraph AckermannController inputs."""
            try:
                _rs = og.Controller.get(og.Controller.attribute(meta["ctrl_attr_speed"]))
                _ra = og.Controller.get(og.Controller.attribute(meta["ctrl_attr_steer"]))
                return (float(_rs) if _rs is not None else 0.0,
                        float(_ra) if _ra is not None else 0.0)
            except Exception:
                return (0.0, 0.0)

        # ── Lazy subscriber-tick discovery ───────────────────────────────────────
        # The opponent vehicle's OmniGraph may not be fully registered in the OG
        # runtime at startup, so og.get_node_by_path() returns invalid nodes and
        # USD GetConnections() may return nothing for the subscriber's execIn.
        # Re-attempt for the first 300 frames so the tick path is found once the
        # graph is fully initialised, then immediately apply TELEOP gating.
        if iteration < 300:
            for _lz_name, _lz_meta in vehicle_teleop_publishers.items():
                if _lz_meta.get("sub_tick_path"):
                    continue  # already found
                _lz_sub = _lz_meta.get("sub_node_path", "")
                if not _lz_sub:
                    # Also retry finding the subscriber node itself
                    for _lz_prim in stage.Traverse():
                        _lz_pp = _lz_prim.GetPath().pathString
                        if not _lz_pp.startswith(f"/World/{_lz_name}"): continue
                        if _lz_prim.GetTypeName() != "OmniGraphNode": continue
                        _lz_nt = ""
                        try:
                            _lz_n = og.get_node_by_path(_lz_pp)
                            if _lz_n and _lz_n.is_valid():
                                _lz_nt = _lz_n.get_node_type().get_node_type()
                        except Exception: pass
                        if not _lz_nt:
                            try: _lz_nt = _lz_prim.GetAttribute("node:type").Get() or ""
                            except Exception: pass
                        if "SubscribeAckermannDrive" in _lz_nt:
                            _lz_meta["sub_node_path"] = _lz_pp
                            _lz_meta["sub_topic_attr"] = f"{_lz_pp}.inputs:topicName"
                            _lz_sub = _lz_pp
                            print(f"\n[Teleop] Lazy: found subscriber for {_lz_name}: {_lz_pp}")
                            _pub_routed_mode.pop(_lz_name, None)
                            break
                if _lz_sub:
                    # Try to find the driving tick via USD connection on inputs:execIn
                    try:
                        _lz_sp = stage.GetPrimAtPath(_lz_sub)
                        _lz_ei = _lz_sp.GetAttribute("inputs:execIn")
                        if _lz_ei:
                            for _lz_src in _lz_ei.GetConnections():
                                _lz_pp2 = _lz_src.GetPrimPath()
                                _lz_tp = stage.GetPrimAtPath(_lz_pp2)
                                if _lz_tp.IsValid():
                                    _lz_tnt = _lz_tp.GetAttribute("node:type").Get() or ""
                                    if any(k in _lz_tnt for k in ("OnPlaybackTick", "OnTick", "SimulationGate", "IsaacSimulationGate")):
                                        _lz_meta["sub_tick_path"] = str(_lz_pp2)
                                        print(f"\n[Teleop] Lazy: found sub_tick for {_lz_name}: {_lz_pp2}")
                                        # Immediately disable if currently in TELEOP
                                        if veh_ctrl_mode.get(_lz_name, "KEYBOARD_CONTROL") == "KEYBOARD_CONTROL":
                                            try:
                                                og.Controller.set(og.Controller.attribute(f"{_lz_pp2}.inputs:enabled"), False)
                                                print(f"\n[Teleop] Lazy: sub_tick DISABLED for {_lz_name}")
                                            except Exception: pass
                                        _pub_routed_mode.pop(_lz_name, None)
                                        break
                    except Exception: pass

        _hud_cmds = {}  # veh_name -> (speed, steer) actually applied this tick
        for _ctrl_veh, _meta in vehicle_teleop_publishers.items():
            _mode = veh_ctrl_mode.get(_ctrl_veh, "KEYBOARD_CONTROL")
            _just_switched_to_ros2 = False
            # In TELEOP mode the ROS2 bridge is never consulted — only keyboard runs.
            if _mode == "ROS2_CONTROL":
                _drv = _drive_cmds.get(_ctrl_veh)
                _ext = _drv if (_drv and (time.monotonic() - float(_drv["stamp"])) < 0.15) else None
            else:
                _ext = None
            if _ctrl_veh == selected_vehicle_name:
                if _mode == "KEYBOARD_CONTROL":
                    _apply_speed, _apply_steer = current_speed, current_steer
                else:  # ROS2 mode
                    if _ext is not None:
                        _apply_speed, _apply_steer = _ext["speed"], _ext["steer"]
                    else:
                        # Bridge has no fresh command — read the actual OmniGraph
                        # controller state so the display reflects the command applied
                        # by the vehicle's built-in ROS2 subscriber (if present).
                        _apply_speed, _apply_steer = _read_og_ctrl(_meta)
                if iteration % max(1, app_freq // 10) == 0:
                    print(f"\r[{_mode}] ACTIVE: {_ctrl_veh} | Spd={_apply_speed:+.2f} m/s, Str={float(_apply_steer) * 57.2958:+.1f}°          ", end="", flush=True)
            else:
                # Reroute subscriber/publisher topics on mode change (non-selected vehicle).
                # Must happen BEFORE continue so TELEOP mode mutes external ROS2 commands.
                if _pub_routed_mode.get(_ctrl_veh) != _mode:
                    _new_pub    = _meta["drive_topic"] if _mode == "KEYBOARD_CONTROL" else _meta["monitor_topic"]
                    _new_sub    = _meta["muted_topic"]  if _mode == "KEYBOARD_CONTROL" else _meta["drive_topic"]
                    _sub_active = (_mode == "ROS2_CONTROL")
                    try:
                        og.Controller.set(og.Controller.attribute(_meta["pub_topic_attr"]), _new_pub)
                        if _meta.get("sub_node_path"):
                            try:
                                og.Controller.set(og.Controller.attribute(f"{_meta['sub_node_path']}.inputs:enabled"), _sub_active)
                            except Exception: pass
                        if _meta.get("sub_tick_path"):
                            try:
                                og.Controller.set(og.Controller.attribute(f"{_meta['sub_tick_path']}.inputs:enabled"), _sub_active)
                            except Exception: pass
                        # Disconnect subscriber→controller data connections in TELEOP so
                        # stale ROS2 values on the connection cannot override keyboard input.
                        # Reconnect in ROS2 mode to restore external drive authority.
                        for _sc_src, _sc_dst in (_meta.get("sub_ctrl_connections") or []):
                            try:
                                if _mode == "KEYBOARD_CONTROL":
                                    og.Controller.disconnect(og.Controller.attribute(_sc_src), og.Controller.attribute(_sc_dst))
                                else:
                                    og.Controller.connect(og.Controller.attribute(_sc_src), og.Controller.attribute(_sc_dst))
                            except Exception: pass
                        if _meta.get("sub_topic_attr"):
                            og.Controller.set(og.Controller.attribute(_meta["sub_topic_attr"]), _new_sub)
                        _pub_routed_mode[_ctrl_veh] = _mode
                        if _mode == "ROS2_CONTROL":
                            _just_switched_to_ros2 = True
                        print(f"\n[Control] {_ctrl_veh}: {'disconnected' if _mode == 'KEYBOARD_CONTROL' else 'reconnected'} subscriber→controller ({_mode})")
                    except Exception as _rr_e:
                        print(f"\n[Teleop] {_ctrl_veh}: failed to reroute: {_rr_e}")
                if _mode == "KEYBOARD_CONTROL":
                    # Non-selected vehicle in TELEOP: hold at 0.0.
                    # We MUST still fall through to the try block so og.Controller.set()
                    # is called every tick — this overrides the subscriber's ROS2 values
                    # which persist on the controller when no set() is issued.
                    _apply_speed, _apply_steer = 0.0, 0.0
                elif _ext is not None:
                    _apply_speed, _apply_steer = _ext["speed"], _ext["steer"]
                else:
                    _apply_speed, _apply_steer = _read_og_ctrl(_meta)
            _hud_cmds[_ctrl_veh] = (_apply_speed, _apply_steer)

            # Reroute subscriber/publisher topics on mode change (selected vehicle path):
            # TELEOP: disable subscriber + mute topic so external ROS2 can't override keyboard
            # ROS2:   re-enable subscriber on drive_topic for external control
            if _pub_routed_mode.get(_ctrl_veh) != _mode:
                _new_pub    = _meta["drive_topic"] if _mode == "KEYBOARD_CONTROL" else _meta["monitor_topic"]
                _new_sub    = _meta["muted_topic"]  if _mode == "KEYBOARD_CONTROL" else _meta["drive_topic"]
                _sub_active = (_mode == "ROS2_CONTROL")
                try:
                    og.Controller.set(og.Controller.attribute(_meta["pub_topic_attr"]), _new_pub)
                    if _meta.get("sub_node_path"):
                        try:
                            og.Controller.set(og.Controller.attribute(f"{_meta['sub_node_path']}.inputs:enabled"), _sub_active)
                        except Exception: pass
                    if _meta.get("sub_tick_path"):
                        try:
                            og.Controller.set(og.Controller.attribute(f"{_meta['sub_tick_path']}.inputs:enabled"), _sub_active)
                        except Exception: pass
                    for _sc_src, _sc_dst in (_meta.get("sub_ctrl_connections") or []):
                        try:
                            if _mode == "KEYBOARD_CONTROL":
                                og.Controller.disconnect(og.Controller.attribute(_sc_src), og.Controller.attribute(_sc_dst))
                            else:
                                og.Controller.connect(og.Controller.attribute(_sc_src), og.Controller.attribute(_sc_dst))
                        except Exception: pass
                    if _meta.get("sub_topic_attr"):
                        og.Controller.set(og.Controller.attribute(_meta["sub_topic_attr"]), _new_sub)
                    _pub_routed_mode[_ctrl_veh] = _mode
                    if _mode == "ROS2_CONTROL":
                        _just_switched_to_ros2 = True
                    print(f"\n[Control] {_ctrl_veh}: {'disconnected' if _mode == 'KEYBOARD_CONTROL' else 'reconnected'} subscriber→controller ({_mode})")
                except Exception as _rr_e:
                    print(f"\n[Teleop] {_ctrl_veh}: failed to reroute: {_rr_e}")

            try:
                if _mode == "KEYBOARD_CONTROL":
                    # TELEOP: drive the controller from keyboard and publish for observability.
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_speed"]), _apply_speed)
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_steer"]), _apply_steer)
                    _pub_p = _meta["pub_node_path"]
                    og.Controller.set(og.Controller.attribute(f"{_pub_p}.inputs:speed"), _apply_speed)
                    og.Controller.set(og.Controller.attribute(f"{_pub_p}.inputs:steeringAngle"), _apply_steer)
                elif _just_switched_to_ros2:
                    # One-shot reset on mode switch: clear the latched keyboard command.
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_speed"]), 0.0)
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_steer"]), 0.0)
                elif _ext is not None:
                    # ROS2 mode with a fresh command: apply it.
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_speed"]), _ext["speed"])
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_steer"]), _ext["steer"])
                else:
                    # ROS2 mode, no command received for >100 ms: safety stop.
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_speed"]), 0.0)
                    og.Controller.set(og.Controller.attribute(_meta["ctrl_attr_steer"]), 0.0)
            except Exception: pass
            
        # Follow Cameras
        for veh_name, cfg in veh_follow_configs.items():
            data = follow_cam_handles.get(veh_name)
            if not data or not data["cam"].GetPrim().IsValid():
                from pxr import UsdGeom
                base_prim = stage.GetPrimAtPath(cfg["base_path"])
                if not base_prim.IsValid(): continue
                # Camera lives at the world level, NOT under the chassis prim.
                # Parenting to the chassis caused one-frame jitter: the physics engine
                # updates the chassis world transform after we write the local transform,
                # so the renderer briefly sees new_chassis × old_local each tick.
                fc_path = f"/World/{cfg['cam_name']}"
                cam_p = stage.GetPrimAtPath(fc_path)
                if not cam_p.IsValid():
                    cam_p = UsdGeom.Camera.Define(stage, fc_path).GetPrim()
                    UsdGeom.Camera(cam_p).GetHorizontalApertureAttr().Set(20.955)
                    UsdGeom.Camera(cam_p).GetVerticalApertureAttr().Set(11.787)
                    UsdGeom.Camera(cam_p).GetFocalLengthAttr().Set(18.1)
                data = {
                    "cam": UsdGeom.Camera(cam_p), "base": base_prim,
                    "front": stage.GetPrimAtPath(cfg["front_path"]) if cfg["front_path"] else None,
                    "rear": stage.GetPrimAtPath(cfg["rear_path"]) if cfg["rear_path"] else None,
                    "dist": cfg["dist"], "height": cfg["height"], "focus_height": cfg["focus_height"],
                    "buf_x":  collections.deque(maxlen=100),
                    "buf_y":  collections.deque(maxlen=100),
                    "buf_z":  collections.deque(maxlen=100),
                    "buf_lz": collections.deque(maxlen=100),
                    # Rotation smoothing: forward-vector components averaged over ~60 frames
                    # (~0.5 s at 120 Hz) to eliminate orientation jitter while still tracking turns
                    "buf_fx": collections.deque(maxlen=60),
                    "buf_fy": collections.deque(maxlen=60),
                    "buf_fz": collections.deque(maxlen=60),
                }
                follow_cam_handles[veh_name] = data

            try:
                from omni.usd import get_world_transform_matrix
                from pxr import Gf, UsdGeom
                m = get_world_transform_matrix(data["base"])
                pos = m.ExtractTranslation()
                rot = m.ExtractRotationMatrix()
                # Derive forward from the chassis rotation matrix only.
                # Using front/rear prim world positions caused vibration when the
                # front wheel steered: the front prim is parented to steering geometry
                # so its world position shifts even when the vehicle body is stationary.
                raw_fwd = rot.GetRow(0)
                up = Gf.Vec3d(0, 0, 1)
                # Smooth position
                data["buf_x"].append(pos[0]); data["buf_y"].append(pos[1])
                data["buf_z"].append(pos[2] + data["height"])
                data["buf_lz"].append(pos[2] + data["focus_height"])
                # Smooth forward vector components to eliminate rotation jitter
                data["buf_fx"].append(raw_fwd[0])
                data["buf_fy"].append(raw_fwd[1])
                data["buf_fz"].append(raw_fwd[2])
                def _avg(b): return sum(b) / len(b)
                sx = _avg(data["buf_x"]); sy = _avg(data["buf_y"])
                sz = _avg(data["buf_z"]); slz = _avg(data["buf_lz"])
                sfwd = Gf.Vec3d(_avg(data["buf_fx"]), _avg(data["buf_fy"]), _avg(data["buf_fz"]))
                sfwd_len = sfwd.GetLength()
                if sfwd_len > 0.01: sfwd = sfwd / sfwd_len
                else: sfwd = raw_fwd
                xy_base = Gf.Vec3d(sx, sy, pos[2]) - (sfwd * data["dist"])
                cam_pos_world = Gf.Vec3d(xy_base[0], xy_base[1], sz)
                lookat_pos = Gf.Vec3d(sx, sy, slz)
                lookat_m_world = Gf.Matrix4d().SetLookAt(cam_pos_world, lookat_pos, up)
                # Camera is at world level so its local transform IS its world transform —
                # no parent inverse needed (and no chassis physics jitter contamination).
                local_m = lookat_m_world.GetInverse()
                xformable = UsdGeom.Xformable(data["cam"])
                x_attr = xformable.GetPrim().GetAttribute("xformOp:transform")
                if not x_attr: xformable.AddTransformOp().Set(local_m)
                else: x_attr.Set(local_m)
            except Exception: pass

        # ── GNSS Publish ──────────────────────────────────────────────────────
        # Runs at _gnss_frame_skip intervals (e.g. every 12 frames at 120 Hz → 10 Hz).
        # Reads the chassis world transform, applies equirectangular projection from
        # the configured map origin to produce WGS-84 lat/lon/alt, adds Gaussian
        # noise, then publishes sensor_msgs/NavSatFix.
        if _gnss_enabled and _gnss_publishers and (iteration % _gnss_frame_skip == 0):
            _gnss_stamp_ns = _gnss_time.time_ns()
            for _gvn, _gpub_info in _gnss_publishers.items():
                try:
                    _gcfg = veh_follow_configs.get(_gvn)
                    if not _gcfg:
                        continue
                    from omni.usd import get_world_transform_matrix as _gwt
                    _gbase = stage.GetPrimAtPath(_gcfg["base_path"])
                    if not _gbase.IsValid():
                        continue
                    _gp = _gwt(_gbase).ExtractTranslation()
                    _gx, _gy, _gz = float(_gp[0]), float(_gp[1]), float(_gp[2])

                    # Equirectangular: sim +X = East (lon), sim +Y = North (lat)
                    _lat = _gnss_lat0 + _gnss_math.degrees(_gy / _R_EARTH)
                    _lon = _gnss_lon0 + _gnss_math.degrees(_gx / (_R_EARTH * _gnss_cos_lat0))
                    _alt = _gnss_alt0 + _gz

                    # Add independent Gaussian noise on each axis
                    _lat += _gnss_math.degrees(_gnss_random.gauss(0.0, _gnss_h_std) / _R_EARTH)
                    _lon += _gnss_math.degrees(_gnss_random.gauss(0.0, _gnss_h_std) / (_R_EARTH * _gnss_cos_lat0))
                    _alt += _gnss_random.gauss(0.0, _gnss_v_std)

                    _gnss_hud_data[_gvn] = (_lat, _lon)

                    # Send to gnss_bridge.py subprocess via stdin pipe.
                    _gline = (f"{_gpub_info['topic']}\t{_gpub_info['frame_id']}\t"
                              f"{_lat}\t{_lon}\t{_alt}\t"
                              f"{_gnss_h_std}\t{_gnss_v_std}\t{_gnss_stamp_ns}\n")
                    _gnss_bridge.stdin.write(_gline)
                    _gnss_bridge.stdin.flush()
                except Exception:
                    pass

        # Update HUD
        if iteration % 4 == 0 and HUD_ENABLED:
            for v_name, v_models in hud_labels.items():
                s_meta = vehicle_sensor_nodes.get(v_name, {})
                # Control source badge reflects the explicit per-vehicle mode (F1/F2 toggle)
                _cs = "CONTROL: ROS2" if veh_ctrl_mode.get(v_name, "KEYBOARD_CONTROL") == "ROS2_CONTROL" else "CONTROL: KEYBOARD"
                if "ctrl_src" in v_models:
                    v_models["ctrl_src"].text = _cs
                    v_models["ctrl_src"].style = {
                        "color": 0xFFFFFFFF,
                        "font_size": FONT_ROW, "font_style": "Bold",
                    }
                # Show actual applied command per vehicle.
                # In TELEOP mode this is the keyboard command; in ROS2 mode it is
                # the last received external command (or 0,0 if stale/absent).
                _cmd = _hud_cmds.get(v_name, (0.0, 0.0))
                v_models["speed"].text = f"{_cmd[0]:+.2f} m/s"
                v_models["steer"].text = f"{float(_cmd[1]) * 57.2958:+.2f}°"
                    
                odom_p = s_meta.get("odom_path")
                if odom_p:
                    try:
                        pos = og.Controller.get(og.Controller.attribute(odom_p + ".outputs:position"))
                        lv  = og.Controller.get(og.Controller.attribute(odom_p + ".outputs:linearVelocity"))
                        av  = og.Controller.get(og.Controller.attribute(odom_p + ".outputs:angularVelocity"))
                        if pos is not None:
                            v_models["pos_x"].text = f"{pos[0]:+.3f} m"
                            v_models["pos_y"].text = f"{pos[1]:+.3f} m"
                            v_models["pos_z"].text = f"{pos[2]:+.3f} m"
                        if lv is not None:
                            v_models["lin_vel"].text = f"{lv[0]:+.2f} m/s"
                        if av is not None:
                            v_models["ang_vel"].text = f"{av[2]:+.2f} r/s"
                    except Exception: pass
                imu_p = s_meta.get("imu_path")
                if imu_p:
                    try:
                        la     = og.Controller.get(og.Controller.attribute(imu_p + ".outputs:linAcc"))
                        av_imu = og.Controller.get(og.Controller.attribute(imu_p + ".outputs:angVel"))
                        if la is not None:
                            v_models["lin_acc"].text = f"{la[0]:+.2f} m/s²"
                        if av_imu is not None:
                            v_models["ang_acc"].text = f"{av_imu[2]:+.2f} r/s"
                    except Exception: pass
                if "gnss_lat" in v_models:
                    _gd = _gnss_hud_data.get(v_name)
                    if _gd is not None:
                        v_models["gnss_lat"].text = f"{_gd[0]:.6f}°"
                        v_models["gnss_lon"].text = f"{_gd[1]:.6f}°"

        # ── Segmentation Capture ──────────────────────────────────────────────
        if is_recording and seg_enabled and seg_rgb_annot and seg_mask_annot:
            if iteration % seg_capture_freq == 0:
                try:
                    # simulation_app.update() already rendered this frame;
                    # annotators pull the latest buffer directly — no orchestrator step needed.
                    _rgb_data = seg_rgb_annot.get_data()
                    _seg_data = seg_mask_annot.get_data()

                    if _rgb_data is not None and _seg_data is not None:
                        # Determine output file number
                        if seg_overwrite:
                            _file_num = seg_capture_index
                        else:
                            _file_num = seg_capture_index
                            while (
                                os.path.exists(os.path.join(seg_images_dir,   f"{_file_num:06d}.png")) or
                                os.path.exists(os.path.join(seg_gt_masks_dir, f"{_file_num:06d}.png"))
                            ):
                                _file_num += 1

                        _img_path  = os.path.join(seg_images_dir,   f"{_file_num:06d}.png")
                        _mask_path = os.path.join(seg_gt_masks_dir, f"{_file_num:06d}.png")

                        # Save RGB image (drop alpha channel if present)
                        _rgb_arr = _np.array(_rgb_data, dtype=_np.uint8)
                        if _rgb_arr.ndim == 3 and _rgb_arr.shape[2] == 4:
                            _rgb_arr = _rgb_arr[:, :, :3]
                        _PILImage.fromarray(_rgb_arr).save(_img_path)
                        os.chmod(_img_path, 0o666)

                        # Build color mask from semantic segmentation data.
                        # seg_data["data"]  : (H, W) uint32 array of replicator-internal IDs
                        # seg_data["info"]["idToLabels"] : {rep_id_str -> {"class": "our_id_str"}}
                        _seg_pixels   = _np.asarray(_seg_data.get("data", _np.zeros((1, 1), dtype=_np.uint32)))
                        _id_to_labels = _seg_data.get("info", {}).get("idToLabels", {})
                        _default_color = seg_color_map.get(seg_default_id, (0, 0, 0))

                        if _seg_pixels.ndim == 3:          # (H, W, 4) packed RGBA id
                            _seg_pixels = _seg_pixels.view(_np.uint32).reshape(_seg_pixels.shape[:2])

                        _h, _w = _seg_pixels.shape[:2]
                        _mask = _np.full((_h, _w, 3), _default_color, dtype=_np.uint8)

                        for _rep_id_str, _label_dict in _id_to_labels.items():
                            try:
                                _our_id = int(_label_dict.get("class", str(seg_default_id)))
                            except (ValueError, TypeError):
                                _our_id = seg_default_id
                            _color = seg_color_map.get(_our_id, _default_color)
                            _mask[_seg_pixels == int(_rep_id_str)] = _color

                        _PILImage.fromarray(_mask).save(_mask_path)
                        os.chmod(_mask_path, 0o666)

                        seg_capture_index += 1
                        print(f"[Record] #{_file_num:06d} | rgb: {_img_path} | mask: {_mask_path}")

                except Exception as _cap_err:
                    import traceback
                    print(f"[Record] Capture error: {_cap_err}")
                    traceback.print_exc()

    simulation_app.close()

if __name__ == "__main__":
    main()
