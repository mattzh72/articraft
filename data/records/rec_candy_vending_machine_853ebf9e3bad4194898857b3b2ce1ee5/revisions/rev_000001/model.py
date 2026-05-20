from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BASE_SIZE = 0.18
BASE_HEIGHT = 0.132
WALL_THICKNESS = 0.003
GLOBE_RADIUS = 0.092
GLOBE_HEIGHT = 0.245
DOOR_WIDTH = 0.094
DOOR_HEIGHT = 0.040
DOOR_THICKNESS = 0.008


def _rounded_panel_mesh(width: float, height: float, thickness: float, name: str):
    radius = min(width, height) * 0.16
    geometry = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius=radius, corner_segments=8),
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geometry, name)


def _globe_shell_mesh():
    outer_profile = [
        (0.086, 0.000),
        (0.090, 0.010),
        (0.092, 0.040),
        (0.092, 0.185),
        (0.086, 0.212),
        (0.060, 0.232),
        (0.024, 0.242),
        (0.000, 0.245),
    ]
    inner_profile = [
        (0.080, 0.006),
        (0.084, 0.016),
        (0.086, 0.044),
        (0.086, 0.184),
        (0.080, 0.208),
        (0.055, 0.226),
        (0.022, 0.234),
        (0.000, 0.237),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=56),
        "candy_globe_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bulk_candy_vending_machine")

    red_enamel = model.material("red_enamel", rgba=(0.73, 0.10, 0.10, 1.0))
    chrome = model.material("chrome", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    clear_acrylic = model.material("clear_acrylic", rgba=(0.76, 0.91, 0.98, 0.30))

    coin_head_main_mesh = _rounded_panel_mesh(0.050, 0.074, 0.028, "coin_head_main")
    coin_head_flange_mesh = _rounded_panel_mesh(0.060, 0.086, 0.008, "coin_head_flange")
    coin_head_bezel_mesh = _rounded_panel_mesh(0.028, 0.050, 0.004, "coin_head_bezel")
    door_panel_mesh = _rounded_panel_mesh(DOOR_HEIGHT, DOOR_WIDTH, DOOR_THICKNESS, "door_panel")
    globe_shell_mesh = _globe_shell_mesh()

    base = model.part("base")
    base.visual(
        Box((BASE_SIZE, BASE_SIZE, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=red_enamel,
        name="bottom_plate",
    )
    base.visual(
        Box((BASE_SIZE, WALL_THICKNESS, 0.126)),
        origin=Origin(xyz=(0.0, 0.5 * (BASE_SIZE - WALL_THICKNESS), 0.067)),
        material=red_enamel,
        name="left_wall",
    )
    base.visual(
        Box((BASE_SIZE, WALL_THICKNESS, 0.126)),
        origin=Origin(xyz=(0.0, -0.5 * (BASE_SIZE - WALL_THICKNESS), 0.067)),
        material=red_enamel,
        name="right_wall",
    )
    base.visual(
        Box((WALL_THICKNESS, BASE_SIZE - 2.0 * WALL_THICKNESS, 0.126)),
        origin=Origin(xyz=(-0.5 * (BASE_SIZE - WALL_THICKNESS), 0.0, 0.067)),
        material=red_enamel,
        name="rear_wall",
    )
    base.visual(
        Box((WALL_THICKNESS, 0.036, 0.126)),
        origin=Origin(xyz=(0.5 * (BASE_SIZE - WALL_THICKNESS), 0.069, 0.067)),
        material=red_enamel,
        name="front_left_jamb",
    )
    base.visual(
        Box((WALL_THICKNESS, 0.036, 0.126)),
        origin=Origin(xyz=(0.5 * (BASE_SIZE - WALL_THICKNESS), -0.069, 0.067)),
        material=red_enamel,
        name="front_right_jamb",
    )
    base.visual(
        Box((WALL_THICKNESS, 0.102, 0.024)),
        origin=Origin(xyz=(0.5 * (BASE_SIZE - WALL_THICKNESS), 0.0, 0.012)),
        material=red_enamel,
        name="front_sill",
    )
    base.visual(
        Box((WALL_THICKNESS, 0.102, 0.034)),
        origin=Origin(xyz=(0.5 * (BASE_SIZE - WALL_THICKNESS), 0.0, 0.113)),
        material=red_enamel,
        name="front_header",
    )
    base.visual(
        Box((BASE_SIZE, BASE_SIZE, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=red_enamel,
        name="top_deck",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
        material=chrome,
        name="globe_support_collar",
    )
    base.visual(
        Box((0.060, 0.070, 0.026)),
        origin=Origin(xyz=(0.060, 0.0, 0.108)),
        material=chrome,
        name="dispenser_body",
    )
    base.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(
            xyz=(0.094, -0.034, 0.028),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="left_hinge_knuckle",
    )
    base.visual(
        Box((0.008, 0.024, 0.008)),
        origin=Origin(xyz=(0.091, -0.034, 0.024)),
        material=chrome,
        name="left_hinge_bracket",
    )
    base.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(
            xyz=(0.094, 0.034, 0.028),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="right_hinge_knuckle",
    )
    base.visual(
        Box((0.008, 0.024, 0.008)),
        origin=Origin(xyz=(0.091, 0.034, 0.024)),
        material=chrome,
        name="right_hinge_bracket",
    )
    for sx in (-0.060, 0.060):
        for sy in (-0.060, 0.060):
            base.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(sx, sy, 0.003)),
                material=dark_metal,
                name=f"foot_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )
    base.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.152)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
    )

    globe = model.part("globe")
    globe.visual(
        globe_shell_mesh,
        material=clear_acrylic,
        name="globe_shell",
    )
    globe.visual(
        Cylinder(radius=0.087, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=chrome,
        name="globe_bottom_band",
    )
    globe.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.242)),
        material=chrome,
        name="globe_top_cap",
    )
    globe.inertial = Inertial.from_geometry(
        Cylinder(radius=GLOBE_RADIUS, length=GLOBE_HEIGHT),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
    )
    model.articulation(
        "base_to_globe",
        ArticulationType.FIXED,
        parent=base,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, 0.152)),
    )

    coin_head = model.part("coin_head")
    coin_head.visual(
        coin_head_flange_mesh,
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="rear_mount_flange",
    )
    coin_head.visual(
        coin_head_main_mesh,
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="head_housing",
    )
    coin_head.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="shaft_boss",
    )
    coin_head.visual(
        coin_head_bezel_mesh,
        origin=Origin(xyz=(0.022, 0.0, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="coin_slot_bezel",
    )
    coin_head.visual(
        Box((0.003, 0.022, 0.003)),
        origin=Origin(xyz=(0.024, 0.0, 0.017)),
        material=knob_black,
        name="coin_slot",
    )
    coin_head.inertial = Inertial.from_geometry(
        Box((0.036, 0.086, 0.060)),
        mass=0.28,
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
    )
    model.articulation(
        "base_to_coin_head",
        ArticulationType.FIXED,
        parent=base,
        child=coin_head,
        origin=Origin(xyz=(0.096, 0.0, 0.095)),
    )

    knob = model.part("dispensing_knob")
    knob.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="knob_hub",
    )
    knob.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_wheel",
    )
    knob.visual(
        Box((0.022, 0.016, 0.010)),
        origin=Origin(xyz=(0.050, 0.0, 0.020)),
        material=chrome,
        name="knob_handle",
    )
    knob.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.059, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="knob_cap",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.052),
        mass=0.12,
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    model.articulation(
        "coin_head_to_knob",
        ArticulationType.CONTINUOUS,
        parent=coin_head,
        child=knob,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    door = model.part("candy_door")
    door.visual(
        Cylinder(radius=0.004, length=0.042),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="door_center_knuckle",
    )
    door.visual(
        door_panel_mesh,
        origin=Origin(
            xyz=(DOOR_THICKNESS, 0.0, 0.5 * DOOR_HEIGHT),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="door_panel",
    )
    door.visual(
        Box((0.012, 0.038, 0.008)),
        origin=Origin(xyz=(0.016, 0.0, 0.020)),
        material=chrome,
        name="door_pull",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.020, 0.094, 0.044)),
        mass=0.18,
        origin=Origin(xyz=(0.010, 0.0, 0.020)),
    )
    model.articulation(
        "base_to_candy_door",
        ArticulationType.REVOLUTE,
        parent=base,
        child=door,
        origin=Origin(xyz=(0.094, 0.0, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    globe = object_model.get_part("globe")
    coin_head = object_model.get_part("coin_head")
    knob = object_model.get_part("dispensing_knob")
    door = object_model.get_part("candy_door")

    knob_joint = object_model.get_articulation("coin_head_to_knob")
    door_joint = object_model.get_articulation("base_to_candy_door")

    ctx.check(
        "all prompt-critical parts exist",
        all(part is not None for part in (base, globe, coin_head, knob, door)),
    )

    ctx.expect_gap(
        globe,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.0015,
        positive_elem="globe_bottom_band",
        negative_elem="globe_support_collar",
        name="globe sits on the square base collar",
    )

    knob_limits = knob_joint.motion_limits
    ctx.check(
        "dispensing knob is continuous on a horizontal front-back shaft",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.axis == (1.0, 0.0, 0.0)
        and knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None,
        details=(
            f"type={knob_joint.articulation_type}, axis={knob_joint.axis}, "
            f"limits={knob_limits}"
        ),
    )

    door_limits = door_joint.motion_limits
    ctx.check(
        "candy door uses a bottom horizontal hinge with downward travel",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and door_joint.axis == (0.0, 1.0, 0.0)
        and door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and door_limits.upper > 1.2,
        details=(
            f"type={door_joint.articulation_type}, axis={door_joint.axis}, "
            f"limits={door_limits}"
        ),
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_pt, max_pt) = aabb
        return tuple((min_pt[i] + max_pt[i]) * 0.5 for i in range(3))

    closed_knob_handle = ctx.part_element_world_aabb(knob, elem="knob_handle")
    with ctx.pose({knob_joint: math.pi / 2.0}):
        turned_knob_handle = ctx.part_element_world_aabb(knob, elem="knob_handle")

    closed_knob_center = _aabb_center(closed_knob_handle)
    turned_knob_center = _aabb_center(turned_knob_handle)
    ctx.check(
        "knob handle visibly sweeps around the shaft",
        closed_knob_center is not None
        and turned_knob_center is not None
        and abs(turned_knob_center[1] - closed_knob_center[1]) > 0.015
        and abs(turned_knob_center[2] - closed_knob_center[2]) > 0.015,
        details=f"closed={closed_knob_center}, turned={turned_knob_center}",
    )

    closed_door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: math.radians(70.0)}):
        opened_door_panel = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "door flap opens outward and downward",
        closed_door_panel is not None
        and opened_door_panel is not None
        and opened_door_panel[1][0] > closed_door_panel[1][0] + 0.020
        and opened_door_panel[1][2] < closed_door_panel[1][2] - 0.010,
        details=f"closed={closed_door_panel}, opened={opened_door_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
