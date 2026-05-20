from math import pi, floor

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


# Dimensions (in meters, realistic linear stage proportions)
BASE_LENGTH = 0.35     # 350mm long
BASE_WIDTH = 0.20      # 200mm wide
BASE_HEIGHT = 0.025    # 25mm thick

RAIL_LENGTH = 0.30     # 300mm rail length
RAIL_WIDTH = 0.02      # 20mm rail width (each rail)
RAIL_HEIGHT = 0.015    # 15mm rail height
RAIL_Y_POS = 0.06      # 60mm from center to each rail

CARRIAGE_LENGTH = 0.10  # 100mm carriage length
CARRIAGE_WIDTH = 0.14   # 140mm carriage width (spans both rails)
CARRIAGE_HEIGHT = 0.04  # 40mm carriage height
CARRIAGE_TRAVEL = 0.18  # 180mm travel range

END_BLOCK_LENGTH = 0.02  # 20mm thick end blocks
END_BLOCK_WIDTH = 0.15   # 150mm wide
END_BLOCK_HEIGHT = 0.03  # 30mm high

# Clamp screw dimensions
SCREW_RADIUS = 0.004    # 4mm radius
SCREW_LENGTH = 0.012    # 12mm length

# Measuring tick dimensions
TICK_HEIGHT = 0.002     # 2mm tall ticks
TICK_WIDTH = 0.001      # 1mm wide
TICK_LENGTH = 0.008     # 8mm long
TICK_SPACING = 0.05     # 50mm spacing between ticks


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_test_stage")
    
    # Define materials with realistic colors
    model.material("base_material", rgba=(0.25, 0.25, 0.28, 1.0))  # Dark gray base
    model.material("rail_material", rgba=(0.70, 0.72, 0.75, 1.0))  # Metallic silver rails
    model.material("carriage_material", rgba=(0.60, 0.63, 0.66, 1.0))  # Lighter metallic
    model.material("screw_material", rgba=(0.10, 0.10, 0.12, 1.0))  # Black screws
    model.material("tick_material", rgba=(0.95, 0.95, 0.30, 1.0))  # Yellow measuring ticks
    model.material("end_block_material", rgba=(0.45, 0.48, 0.50, 1.0))  # Medium gray
    
    # --- BASE ---
    # Base is centered at (BASE_LENGTH/2, BASE_WIDTH/2, 0) with bottom at z=0
    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        origin=Origin(xyz=(BASE_LENGTH/2, BASE_WIDTH/2, BASE_HEIGHT/2)),
        material="base_material",
        name="base_plate",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        mass=2.5,
        origin=Origin(xyz=(BASE_LENGTH/2, BASE_WIDTH/2, BASE_HEIGHT/2)),
    )
    
    # --- GUIDE RAILS ---
    # Rails sit on top of base (z=BASE_HEIGHT) and run along X-axis
    # Rail part frame is at the center of the rail
    for i, y_pos in enumerate([-RAIL_Y_POS, RAIL_Y_POS]):
        rail = model.part(f"rail_{i}")
        # Rail visual: centered at (RAIL_LENGTH/2, 0, RAIL_HEIGHT/2) relative to rail part frame
        rail.visual(
            Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
            origin=Origin(xyz=(RAIL_LENGTH/2, 0.0, RAIL_HEIGHT/2)),
            material="rail_material",
            name=f"rail_{i}_body",
        )
        rail.inertial = Inertial.from_geometry(
            Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
            mass=0.4,
            origin=Origin(xyz=(RAIL_LENGTH/2, 0.0, RAIL_HEIGHT/2)),
        )
        
        # Mount rails to base with FIXED articulation
        # Rail part frame position in world: (BASE_LENGTH/2, BASE_WIDTH/2 + y_pos, BASE_HEIGHT)
        # But we want the rail to be centered at (BASE_LENGTH/2, BASE_WIDTH/2 + y_pos, BASE_HEIGHT + RAIL_HEIGHT/2)
        # So the rail part frame should be at (BASE_LENGTH/2, BASE_WIDTH/2 + y_pos, BASE_HEIGHT + RAIL_HEIGHT/2)
        # Wait, no. The rail visual is at (RAIL_LENGTH/2, 0, RAIL_HEIGHT/2) relative to rail part frame.
        # So if rail part frame is at (x, y, z), the rail center is at (x + RAIL_LENGTH/2, y, z + RAIL_HEIGHT/2).
        # We want the rail center at (BASE_LENGTH/2, BASE_WIDTH/2 + y_pos, BASE_HEIGHT + RAIL_HEIGHT/2).
        # So: x + RAIL_LENGTH/2 = BASE_LENGTH/2 => x = BASE_LENGTH/2 - RAIL_LENGTH/2
        #     y = BASE_WIDTH/2 + y_pos
        #     z + RAIL_HEIGHT/2 = BASE_HEIGHT + RAIL_HEIGHT/2 => z = BASE_HEIGHT
        
        model.articulation(
            f"base_to_rail_{i}",
            ArticulationType.FIXED,
            parent=base,
            child=rail,
            origin=Origin(xyz=(
                BASE_LENGTH/2 - RAIL_LENGTH/2,  # x: center rail along X on base
                BASE_WIDTH/2 + y_pos,            # y: offset from base center
                BASE_HEIGHT,                      # z: on top of base
            )),
        )
    
    # --- CARRIAGE ---
    # Carriage sits on top of rails
    # Carriage part frame will be set by the prismatic joint
    # Visual origin is relative to part frame: center carriage on part frame
    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(CARRIAGE_LENGTH/2, 0.0, CARRIAGE_HEIGHT/2)),
        material="carriage_material",
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=0.8,
        origin=Origin(xyz=(CARRIAGE_LENGTH/2, 0.0, CARRIAGE_HEIGHT/2)),
    )
    
    # Add clamp screws to carriage (on top surface)
    # Positions relative to carriage part frame (Y=0 is center of carriage)
    screw_positions = [
        (CARRIAGE_LENGTH * 0.25, -CARRIAGE_WIDTH * 0.25),
        (CARRIAGE_LENGTH * 0.75, -CARRIAGE_WIDTH * 0.25),
        (CARRIAGE_LENGTH * 0.25, CARRIAGE_WIDTH * 0.25),
        (CARRIAGE_LENGTH * 0.75, CARRIAGE_WIDTH * 0.25),
    ]
    for j, (x, y) in enumerate(screw_positions):
        carriage.visual(
            Cylinder(radius=SCREW_RADIUS, length=SCREW_LENGTH),
            origin=Origin(
                xyz=(x, y, CARRIAGE_HEIGHT + SCREW_LENGTH/2),
                rpy=(0.0, pi/2, 0.0),  # Cylinder along Z
            ),
            material="screw_material",
            name=f"clamp_screw_{j}",
        )
    
    # Add measuring ticks along the carriage (on top surface, along X-axis)
    # Ticks positioned relative to carriage part frame
    num_ticks = int(CARRIAGE_LENGTH / TICK_SPACING)
    for k in range(num_ticks + 1):
        x_pos = k * TICK_SPACING
        if x_pos > CARRIAGE_LENGTH:
            x_pos = CARRIAGE_LENGTH
        carriage.visual(
            Box((TICK_LENGTH, TICK_WIDTH, TICK_HEIGHT)),
            origin=Origin(xyz=(x_pos, 0.0, CARRIAGE_HEIGHT + TICK_HEIGHT/2)),
            material="tick_material",
            name=f"measuring_tick_{k}",
        )
    
    # --- PRISMATIC ARTICULATION: Base to Carriage ---
    # The carriage should slide along the X-axis on top of the rails
    # At q=0, carriage part frame = articulation frame
    # We want carriage center at (BASE_LENGTH/2, BASE_WIDTH/2, BASE_HEIGHT + RAIL_HEIGHT + CARRIAGE_HEIGHT/2) at q=0
    # Carriage part frame should be at (BASE_LENGTH/2 - CARRIAGE_LENGTH/2, BASE_WIDTH/2 - CARRIAGE_WIDTH/2, BASE_HEIGHT + RAIL_HEIGHT)
    # Because carriage visual is at (CARRIAGE_LENGTH/2, CARRIAGE_WIDTH/2, CARRIAGE_HEIGHT/2) relative to part frame
    
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        # Articulation frame: center of carriage travel range
        origin=Origin(xyz=(
            BASE_LENGTH/2 - CARRIAGE_LENGTH/2,  # x: center of carriage at q=0
            BASE_WIDTH/2,                        # y: centered on base
            BASE_HEIGHT + RAIL_HEIGHT,           # z: on top of rails (carriage bottom)
        )),
        axis=(1.0, 0.0, 0.0),  # Slide along X-axis
        motion_limits=MotionLimits(
            lower=-CARRIAGE_TRAVEL/2,
            upper=CARRIAGE_TRAVEL/2,
            effort=500.0,
            velocity=0.3,
        ),
    )
    
    # --- END BLOCKS ---
    # Left end block (at negative travel limit)
    end_block_left = model.part("end_block_left")
    end_block_left.visual(
        Box((END_BLOCK_LENGTH, END_BLOCK_WIDTH, END_BLOCK_HEIGHT)),
        origin=Origin(xyz=(END_BLOCK_LENGTH/2, END_BLOCK_WIDTH/2, END_BLOCK_HEIGHT/2)),
        material="end_block_material",
        name="end_block_left_body",
    )
    model.articulation(
        "base_to_end_block_left",
        ArticulationType.FIXED,
        parent=base,
        child=end_block_left,
        origin=Origin(xyz=(
            BASE_LENGTH/2 - RAIL_LENGTH/2 - END_BLOCK_LENGTH,  # x: left of rails
            BASE_WIDTH/2 - END_BLOCK_WIDTH/2,                  # y: centered
            BASE_HEIGHT,                                       # z: on top of base
        )),
    )
    
    # Right end block (at positive travel limit)
    end_block_right = model.part("end_block_right")
    end_block_right.visual(
        Box((END_BLOCK_LENGTH, END_BLOCK_WIDTH, END_BLOCK_HEIGHT)),
        origin=Origin(xyz=(END_BLOCK_LENGTH/2, END_BLOCK_WIDTH/2, END_BLOCK_HEIGHT/2)),
        material="end_block_material",
        name="end_block_right_body",
    )
    model.articulation(
        "base_to_end_block_right",
        ArticulationType.FIXED,
        parent=base,
        child=end_block_right,
        origin=Origin(xyz=(
            BASE_LENGTH/2 + RAIL_LENGTH/2,  # x: right of rails
            BASE_WIDTH/2 - END_BLOCK_WIDTH/2,
            BASE_HEIGHT,
        )),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    rail_0 = object_model.get_part("rail_0")
    rail_1 = object_model.get_part("rail_1")
    joint = object_model.get_articulation("base_to_carriage")
    
    # --- Test 1: Carriage overlaps with rails in Y (spans both rails) ---
    # The carriage spans both rails in Y, so check Y overlap (not Z, since they only touch at a plane)
    ctx.expect_overlap(
        carriage,
        rail_0,
        axes="y",
        min_overlap=0.01,
        name="carriage overlaps rail_0 in Y at rest",
    )
    ctx.expect_overlap(
        carriage,
        rail_1,
        axes="y",
        min_overlap=0.01,
        name="carriage overlaps rail_1 in Y at rest",
    )
    
    # --- Test 2: Carriage overlaps with rails at rest (X-axis) ---
    ctx.expect_overlap(
        carriage,
        rail_0,
        axes="x",
        min_overlap=CARRIAGE_LENGTH * 0.5,
        name="carriage overlaps rail_0 at rest in X",
    )
    ctx.expect_overlap(
        carriage,
        rail_1,
        axes="x",
        min_overlap=CARRIAGE_LENGTH * 0.5,
        name="carriage overlaps rail_1 at rest in X",
    )
    
    # --- Test 3: Carriage extends correctly along +X ---
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({joint: joint.motion_limits.upper}):
        ctx.expect_overlap(
            carriage,
            rail_0,
            axes="y",
            min_overlap=0.01,
            name="extended carriage overlaps rail_0 in Y",
        )
        ctx.expect_overlap(
            carriage,
            rail_1,
            axes="y",
            min_overlap=0.01,
            name="extended carriage overlaps rail_1 in Y",
        )
        ctx.expect_overlap(
            carriage,
            rail_0,
            axes="x",
            min_overlap=CARRIAGE_LENGTH * 0.2,
            name="extended carriage retains insertion in rail_0",
        )
        ctx.expect_overlap(
            carriage,
            rail_1,
            axes="x",
            min_overlap=CARRIAGE_LENGTH * 0.2,
            name="extended carriage retains insertion in rail_1",
        )
        extended_pos = ctx.part_world_position(carriage)
    
    ctx.check(
        "carriage extends along +X",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.05,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    
    # --- Test 4: Carriage retracts correctly along -X ---
    with ctx.pose({joint: joint.motion_limits.lower}):
        ctx.expect_overlap(
            carriage,
            rail_0,
            axes="y",
            min_overlap=0.01,
            name="retracted carriage overlaps rail_0 in Y",
        )
        ctx.expect_overlap(
            carriage,
            rail_1,
            axes="y",
            min_overlap=0.01,
            name="retracted carriage overlaps rail_1 in Y",
        )
        ctx.expect_overlap(
            carriage,
            rail_0,
            axes="x",
            min_overlap=CARRIAGE_LENGTH * 0.2,
            name="retracted carriage retains insertion in rail_0",
        )
        ctx.expect_overlap(
            carriage,
            rail_1,
            axes="x",
            min_overlap=CARRIAGE_LENGTH * 0.2,
            name="retracted carriage retains insertion in rail_1",
        )
        retracted_pos = ctx.part_world_position(carriage)
    
    ctx.check(
        "carriage retracts along -X",
        rest_pos is not None and retracted_pos is not None and retracted_pos[0] < rest_pos[0] - 0.05,
        details=f"rest={rest_pos}, retracted={retracted_pos}",
    )
    
    # --- Test 5: Verify visible features (screws and ticks) are present ---
    carriage_visuals = [v.name for v in carriage.visuals if v.name]
    screw_count = sum(1 for name in carriage_visuals if "clamp_screw" in name)
    tick_count = sum(1 for name in carriage_visuals if "measuring_tick" in name)
    
    ctx.check(
        "carriage has all clamp screws",
        screw_count == 4,
        details=f"Found {screw_count} screws, expected 4",
    )
    ctx.check(
        "carriage has measuring ticks",
        tick_count >= 2,
        details=f"Found {tick_count} ticks",
    )
    
    return ctx.report()


object_model = build_object_model()
