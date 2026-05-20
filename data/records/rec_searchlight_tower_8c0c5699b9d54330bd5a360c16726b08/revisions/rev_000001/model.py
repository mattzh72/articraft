from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def _cylinder_between(part, name, start, end, radius, material):
    """Add a cylinder visual running from start to end in the part frame."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    horizontal = math.sqrt(vx * vx + vy * vy)
    pitch = math.atan2(horizontal, vz)
    yaw = math.atan2(vy, vx) if horizontal > 1e-9 else 0.0
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _bolt(part, name, xyz, radius, height, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=height),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_searchlight_tower")

    steel = model.material("dark_galvanized_steel", color=(0.16, 0.17, 0.16, 1.0))
    black = model.material("matte_black_housing", color=(0.02, 0.022, 0.02, 1.0))
    hazard = model.material("safety_yellow_powdercoat", color=(1.0, 0.72, 0.04, 1.0))
    red = model.material("red_lockout_handles", color=(0.85, 0.04, 0.03, 1.0))
    glass = model.material("blue_tinted_lens", color=(0.38, 0.68, 0.95, 0.55))
    fastener = model.material("bright_fasteners", color=(0.62, 0.64, 0.60, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((1.20, 0.90, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=steel,
        name="base_plate",
    )
    tower.visual(
        Box((0.78, 0.52, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0975)),
        material=hazard,
        name="mast_foot_plate",
    )
    tower.visual(
        Cylinder(radius=0.083, length=2.16),
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
        material=steel,
        name="round_mast",
    )
    tower.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.31)),
        material=steel,
        name="fixed_bearing",
    )
    tower.visual(
        Cylinder(radius=0.38, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 2.3275)),
        material=steel,
        name="index_plate",
    )

    for i, (x, y) in enumerate(((-0.48, -0.33), (0.48, -0.33), (-0.48, 0.33), (0.48, 0.33))):
        tower.visual(
            Box((0.18, 0.13, 0.022)),
            origin=Origin(xyz=(x, y, 0.091)),
            material=steel,
            name=f"anchor_pad_{i}",
        )
        _bolt(tower, f"anchor_bolt_{i}", (x, y, 0.108), 0.026, 0.018, fastener)

    brace_points = [
        ((-0.38, -0.28, 0.12), (-0.055, -0.035, 1.46)),
        ((0.38, -0.28, 0.12), (0.055, -0.035, 1.46)),
        ((-0.38, 0.28, 0.12), (-0.055, 0.035, 1.46)),
        ((0.38, 0.28, 0.12), (0.055, 0.035, 1.46)),
    ]
    for i, (a, b) in enumerate(brace_points):
        _cylinder_between(tower, f"mast_brace_{i}", a, b, 0.025, hazard)

    # Fixed over-travel stop posts, tied into the top indexing plate.
    tower.visual(
        Box((0.11, 0.07, 0.14)),
        origin=Origin(xyz=(-0.16, -0.36, 2.405)),
        material=hazard,
        name="pan_stop_0",
    )
    tower.visual(
        Box((0.11, 0.07, 0.14)),
        origin=Origin(xyz=(0.16, -0.36, 2.405)),
        material=hazard,
        name="pan_stop_1",
    )
    for i, (x, y) in enumerate(((-0.12, -0.26), (0.12, -0.26), (-0.12, 0.26), (0.12, 0.26))):
        _bolt(tower, f"bearing_bolt_{i}", (x, y, 2.350), 0.018, 0.018, fastener)

    pan = model.part("pan_carriage")
    pan.visual(
        Cylinder(radius=0.245, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=steel,
        name="turntable_disk",
    )
    pan.visual(
        Box((0.90, 0.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=hazard,
        name="yoke_saddle",
    )
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.92, 0.36, 0.78),
            span_width=0.58,
            trunnion_diameter=0.11,
            trunnion_center_z=0.55,
            base_thickness=0.14,
            corner_radius=0.018,
            center=False,
        ),
        "heavy_trunnion_yoke",
    )
    pan.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=hazard,
        name="trunnion_yoke",
    )
    for i, x in enumerate((-0.30, 0.30)):
        pan.visual(
            Box((0.38, 0.05, 0.25)),
            origin=Origin(xyz=(x, -0.205, 0.265)),
            material=steel,
            name=f"rear_cross_tie_{i}",
        )
    pan.visual(
        Box((0.96, 0.05, 0.21)),
        origin=Origin(xyz=(0.0, 0.205, 0.245)),
        material=steel,
        name="front_cross_tie",
    )
    # Heavy diagonal load paths from the rotating saddle into both yoke cheeks.
    for i, x in enumerate((-0.37, 0.37)):
        _cylinder_between(pan, f"yoke_gusset_{i}_rear", (0.12 * (1 if x > 0 else -1), -0.15, 0.14), (x, -0.10, 0.47), 0.022, steel)
        _cylinder_between(pan, f"yoke_gusset_{i}_front", (0.12 * (1 if x > 0 else -1), 0.15, 0.14), (x, 0.10, 0.47), 0.022, steel)

    pan.visual(
        Box((0.08, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.25, 0.09)),
        material=hazard,
        name="rotating_stop_lug",
    )
    # Lockout guide brackets are open around the red sliding pin.
    pan.visual(
        Box((0.17, 0.035, 0.06)),
        origin=Origin(xyz=(0.0, -0.245, 0.19)),
        material=steel,
        name="lock_guide_top",
    )
    pan.visual(
        Box((0.17, 0.035, 0.06)),
        origin=Origin(xyz=(0.0, -0.245, 0.08)),
        material=steel,
        name="lock_guide_bottom",
    )
    pan.visual(
        Box((0.04, 0.035, 0.17)),
        origin=Origin(xyz=(-0.085, -0.245, 0.135)),
        material=steel,
        name="lock_guide_side_0",
    )
    pan.visual(
        Box((0.04, 0.035, 0.17)),
        origin=Origin(xyz=(0.085, -0.245, 0.135)),
        material=steel,
        name="lock_guide_side_1",
    )
    for i, x in enumerate((-0.12, 0.12)):
        pan.visual(
            Box((0.035, 0.07, 0.20)),
            origin=Origin(xyz=(x, -0.230, 0.135)),
            material=steel,
            name=f"lock_guide_web_{i}",
        )

    for i, (x, y, z) in enumerate(
        (
            (-0.468, -0.12, 0.36),
            (0.468, -0.12, 0.36),
            (-0.468, 0.12, 0.36),
            (0.468, 0.12, 0.36),
            (-0.468, -0.12, 0.66),
            (0.468, -0.12, 0.66),
            (-0.468, 0.12, 0.66),
            (0.468, 0.12, 0.66),
        )
    ):
        _bolt(pan, f"yoke_bolt_{i}", (x, y, z), 0.018, 0.016, fastener, rpy=(0.0, math.pi / 2.0, 0.0))

    head = model.part("lamp_head")
    barrel = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.165, -0.28), (0.205, -0.22), (0.235, 0.19), (0.245, 0.31)],
        inner_profile=[(0.115, -0.25), (0.155, -0.19), (0.182, 0.17), (0.195, 0.285)],
        segments=64,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    head.visual(
        mesh_from_geometry(barrel, "lamp_shell_mesh"),
        origin=Origin(xyz=(0.0, 0.03, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="lamp_shell",
    )
    head.visual(
        Cylinder(radius=0.206, length=0.03),
        origin=Origin(xyz=(0.0, 0.335, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    head.visual(
        Cylinder(radius=0.255, length=0.035),
        origin=Origin(xyz=(0.0, 0.325, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_bezel",
    )
    head.visual(
        Box((0.54, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, 0.355, 0.0)),
        material=steel,
        name="guard_bar_horizontal",
    )
    head.visual(
        Box((0.026, 0.022, 0.54)),
        origin=Origin(xyz=(0.0, 0.355, 0.0)),
        material=steel,
        name="guard_bar_vertical",
    )
    for i, x in enumerate((-0.17, 0.17)):
        head.visual(
            Box((0.022, 0.022, 0.45)),
            origin=Origin(xyz=(x, 0.358, 0.0)),
            material=steel,
            name=f"guard_bar_{i}",
        )
    head.visual(
        Box((0.26, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, -0.30, 0.0)),
        material=steel,
        name="rear_junction_box",
    )
    head.visual(
        Cylinder(radius=0.057, length=0.94),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tilt_trunnion_pin",
    )
    head.visual(
        Box((0.48, 0.08, 0.09)),
        origin=Origin(xyz=(0.0, -0.03, -0.205)),
        material=steel,
        name="trunnion_tie_bar",
    )
    # Over-travel ears welded to the lamp body, placed to meet yoke stops before cable strain.
    head.visual(
        Box((0.08, 0.06, 0.13)),
        origin=Origin(xyz=(-0.21, -0.08, 0.16)),
        material=hazard,
        name="tilt_stop_0",
    )
    head.visual(
        Box((0.08, 0.06, 0.13)),
        origin=Origin(xyz=(0.21, -0.08, 0.16)),
        material=hazard,
        name="tilt_stop_1",
    )

    lock_pin = model.part("pan_lock_pin")
    lock_pin.visual(
        Cylinder(radius=0.019, length=0.22),
        origin=Origin(xyz=(0.0, -0.07, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="lock_pin_shaft",
    )
    lock_pin.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(xyz=(0.0, -0.17, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="pull_handle",
    )

    tilt_lock_knob = model.part("tilt_lock_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.085,
            0.055,
            body_style="lobed",
            base_diameter=0.060,
            top_diameter=0.078,
            grip=KnobGrip(style="ribbed", count=8, depth=0.003),
            center=False,
        ),
        "tilt_lock_knob_mesh",
    )
    tilt_lock_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="knob_body",
    )

    model.articulation(
        "tower_to_pan",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=pan,
        origin=Origin(xyz=(0.0, 0.0, 2.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.7, lower=-2.45, upper=2.45),
    )
    model.articulation(
        "pan_to_lamp",
        ArticulationType.REVOLUTE,
        parent=pan,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.61)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.65, lower=-0.75, upper=0.85),
    )
    model.articulation(
        "pan_to_lock_pin",
        ArticulationType.PRISMATIC,
        parent=pan,
        child=lock_pin,
        origin=Origin(xyz=(0.0, -0.245, 0.135)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.18, lower=0.0, upper=0.08),
    )
    model.articulation(
        "pan_to_tilt_lock",
        ArticulationType.REVOLUTE,
        parent=pan,
        child=tilt_lock_knob,
        origin=Origin(xyz=(0.46, -0.16, 0.61)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-math.pi, upper=math.pi),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    tower = object_model.get_part("tower")
    pan = object_model.get_part("pan_carriage")
    head = object_model.get_part("lamp_head")
    lock_pin = object_model.get_part("pan_lock_pin")
    pan_joint = object_model.get_articulation("tower_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_lamp")
    pin_joint = object_model.get_articulation("pan_to_lock_pin")

    ctx.allow_overlap(
        head,
        pan,
        elem_a="tilt_trunnion_pin",
        elem_b="trunnion_yoke",
        reason="The oversized heavy-duty trunnion pin is intentionally captured in the yoke bearing bores.",
    )

    ctx.expect_gap(
        pan,
        tower,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="fixed_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan bearing stack is seated without interpenetration",
    )
    ctx.expect_overlap(
        head,
        pan,
        axes="x",
        elem_a="tilt_trunnion_pin",
        elem_b="trunnion_yoke",
        min_overlap=0.60,
        name="tilt trunnion spans both yoke bearing blocks",
    )
    ctx.expect_within(
        head,
        pan,
        axes="x",
        inner_elem="lamp_shell",
        outer_elem="trunnion_yoke",
        margin=0.01,
        name="lamp shell sits inside yoke clear span",
    )
    ctx.expect_overlap(
        lock_pin,
        pan,
        axes="y",
        elem_a="lock_pin_shaft",
        elem_b="lock_guide_top",
        min_overlap=0.02,
        name="lockout pin remains captured in its guide",
    )

    rest_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({tilt_joint: 0.65}):
        raised_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    ctx.check(
        "positive tilt raises the searchlight beam",
        rest_lens is not None
        and raised_lens is not None
        and raised_lens[1][2] > rest_lens[1][2] + 0.10,
        details=f"rest={rest_lens}, raised={raised_lens}",
    )

    rest_pin = ctx.part_world_position(lock_pin)
    with ctx.pose({pin_joint: 0.08}):
        pulled_pin = ctx.part_world_position(lock_pin)
    ctx.check(
        "lockout pin retracts along its guide",
        rest_pin is not None and pulled_pin is not None and pulled_pin[1] < rest_pin[1] - 0.06,
        details=f"rest={rest_pin}, pulled={pulled_pin}",
    )

    with ctx.pose({pan_joint: 0.75}):
        ctx.expect_gap(
            pan,
            tower,
            axis="z",
            positive_elem="turntable_disk",
            negative_elem="fixed_bearing",
            max_gap=0.001,
            max_penetration=0.0,
            name="pan bearing remains vertically seated while slewing",
        )

    return ctx.report()


object_model = build_object_model()
