from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibrated_studio_spotlight")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.34, 0.35, 0.34, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    datum_white = model.material("datum_white", rgba=(0.92, 0.91, 0.84, 1.0))
    amber = model.material("warm_lens", rgba=(0.90, 0.63, 0.24, 0.55))

    # Root assembly: a single mechanically continuous stack from base to yoke.
    stand = model.part("stand")
    stand.visual(
        Box((0.58, 0.42, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_metal,
        name="base_plate",
    )
    stand.visual(
        Box((0.40, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=datum_white,
        name="base_x_datum",
    )
    stand.visual(
        Box((0.010, 0.30, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=datum_white,
        name="base_y_datum",
    )
    stand.visual(
        Cylinder(radius=0.036, length=0.645),
        origin=Origin(xyz=(0.0, 0.0, 0.362), rpy=(0.0, 0.0, 0.0)),
        material=satin_metal,
        name="mast",
    )
    stand.visual(
        Cylinder(radius=0.058, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.655)),
        material=dark_metal,
        name="top_collar",
    )
    stand.visual(
        Box((0.095, 0.090, 0.090)),
        origin=Origin(xyz=(-0.050, 0.0, 0.675)),
        material=dark_metal,
        name="yoke_riser",
    )
    stand.visual(
        Box((0.120, 0.410, 0.060)),
        origin=Origin(xyz=(-0.040, 0.0, 0.680)),
        material=dark_metal,
        name="lower_bridge",
    )

    inner_y = 0.166
    cheek_thickness = 0.039
    cheek_outer_y = inner_y + cheek_thickness
    cheek_center_y = inner_y + cheek_thickness / 2.0
    stand.visual(
        Box((0.180, cheek_thickness, 0.440)),
        origin=Origin(xyz=(0.0, cheek_center_y, 0.900)),
        material=dark_metal,
        name="yoke_cheek_0",
    )
    stand.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(
            xyz=(0.0, cheek_outer_y + 0.004, 0.900),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_metal,
        name="lock_boss_0",
    )
    stand.visual(
        Box((0.180, cheek_thickness, 0.440)),
        origin=Origin(xyz=(0.0, -cheek_center_y, 0.900)),
        material=dark_metal,
        name="yoke_cheek_1",
    )
    stand.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(
            xyz=(0.0, -(cheek_outer_y + 0.004), 0.900),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_metal,
        name="lock_boss_1",
    )

    # Repeatable alignment scale on the side yoke: raised tick marks are
    # slightly seated into the cheek face so they read as painted/etched datum
    # marks rather than separate floating pieces.
    for i, angle_deg in enumerate((-60, -40, -20, 0, 20, 40, 60)):
        theta = math.radians(angle_deg)
        radius = 0.086
        tick_len = 0.034 if angle_deg in (-60, 0, 60) else 0.024
        stand.visual(
            Box((tick_len, 0.003, 0.006)),
            origin=Origin(
                xyz=(
                    radius * math.sin(theta),
                    cheek_outer_y + 0.001,
                    0.900 + radius * math.cos(theta),
                ),
                rpy=(0.0, theta - math.pi / 2.0, 0.0),
            ),
            material=datum_white,
            name=f"tilt_index_{i}",
        )
    stand.visual(
        Box((0.012, 0.004, 0.145)),
        origin=Origin(xyz=(0.0, cheek_outer_y + 0.001, 0.990)),
        material=datum_white,
        name="zero_datum_line",
    )

    can = model.part("can")
    shell_profile_outer = [
        (0.130, -0.205),
        (0.145, -0.185),
        (0.145, 0.160),
        (0.164, 0.190),
        (0.164, 0.242),
    ]
    shell_profile_inner = [
        (0.112, -0.192),
        (0.132, -0.172),
        (0.132, 0.175),
        (0.146, 0.198),
        (0.146, 0.232),
    ]
    can_shell = LatheGeometry.from_shell_profiles(
        shell_profile_outer,
        shell_profile_inner,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)
    can.visual(
        mesh_from_geometry(can_shell, "can_shell"),
        material=matte_black,
        name="can_shell",
    )
    can.visual(
        Cylinder(radius=0.148, length=0.007),
        origin=Origin(xyz=(0.229, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber,
        name="front_lens",
    )
    can.visual(
        Cylinder(radius=0.132, length=0.020),
        origin=Origin(xyz=(-0.198, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_cap",
    )
    can.visual(
        Box((0.260, 0.028, 0.012)),
        origin=Origin(xyz=(0.035, 0.0, 0.150)),
        material=satin_metal,
        name="top_datum_rail",
    )
    can.visual(
        Cylinder(radius=0.037, length=0.332),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="trunnion_axle",
    )
    for suffix, sign in (("0", 1.0), ("1", -1.0)):
        can.visual(
            Cylinder(radius=0.058, length=0.014),
            origin=Origin(
                xyz=(0.0, sign * 0.154, 0.0),
                rpy=(-sign * math.pi / 2.0, 0.0, 0.0),
            ),
            material=satin_metal,
            name=f"friction_disc_{suffix}",
        )
    can.visual(
        Box((0.014, 0.004, 0.080)),
        origin=Origin(xyz=(0.0, 0.159, 0.078)),
        material=datum_white,
        name="scale_pointer",
    )

    lock_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.066,
            0.035,
            body_style="lobed",
            base_diameter=0.050,
            top_diameter=0.060,
            crown_radius=0.001,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0012),
            bore=KnobBore(style="round", diameter=0.010),
            center=False,
        ),
        "tilt_lock_knob",
    )
    tilt_lock_0 = model.part("tilt_lock_0")
    tilt_lock_0.visual(
        lock_knob_mesh,
        material=matte_black,
        name="lock_knob",
    )
    tilt_lock_1 = model.part("tilt_lock_1")
    tilt_lock_1.visual(
        lock_knob_mesh,
        material=matte_black,
        name="lock_knob",
    )

    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=-0.75, upper=0.75),
        motion_properties=MotionProperties(damping=0.25, friction=2.2),
    )
    for suffix, sign, lock_part in (("0", 1.0, tilt_lock_0), ("1", -1.0, tilt_lock_1)):
        model.articulation(
            f"stand_to_tilt_lock_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=stand,
            child=lock_part,
            origin=Origin(
                xyz=(0.0, sign * (cheek_outer_y + 0.010), 0.900),
                rpy=(-sign * math.pi / 2.0, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=6.0),
            motion_properties=MotionProperties(damping=0.05, friction=0.4),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    can = object_model.get_part("can")
    tilt_lock_0 = object_model.get_part("tilt_lock_0")
    tilt_lock_1 = object_model.get_part("tilt_lock_1")
    tilt = object_model.get_articulation("tilt")
    lock_joint_0 = object_model.get_articulation("stand_to_tilt_lock_0")
    lock_joint_1 = object_model.get_articulation("stand_to_tilt_lock_1")

    ctx.check(
        "tilt is yoke-axis limited",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.axis == (0.0, 1.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < -0.70
        and tilt.motion_limits.upper > 0.70,
        details=f"tilt={tilt}",
    )
    ctx.check(
        "both lock knobs spin on their clamp axes",
        lock_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and lock_joint_1.articulation_type == ArticulationType.CONTINUOUS
        and lock_joint_0.axis == (0.0, 0.0, 1.0)
        and lock_joint_1.axis == (0.0, 0.0, 1.0),
        details=f"lock_joint_0={lock_joint_0}, lock_joint_1={lock_joint_1}",
    )

    with ctx.pose({tilt: 0.0}):
        ctx.expect_gap(
            stand,
            can,
            axis="y",
            positive_elem="yoke_cheek_0",
            negative_elem="trunnion_axle",
            max_gap=0.001,
            max_penetration=0.0,
            name="positive yoke cheek captures trunnion",
        )
        ctx.expect_gap(
            can,
            stand,
            axis="y",
            positive_elem="trunnion_axle",
            negative_elem="yoke_cheek_1",
            max_gap=0.001,
            max_penetration=0.0,
            name="negative yoke cheek captures trunnion",
        )
        ctx.expect_gap(
            can,
            stand,
            axis="z",
            positive_elem="can_shell",
            negative_elem="lower_bridge",
            min_gap=0.020,
            max_gap=0.040,
            name="can clears lower yoke bridge",
        )
        ctx.expect_gap(
            tilt_lock_0,
            stand,
            axis="y",
            negative_elem="lock_boss_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="positive lock knob seats on boss",
        )
        ctx.expect_gap(
            stand,
            tilt_lock_1,
            axis="y",
            positive_elem="lock_boss_1",
            max_gap=0.001,
            max_penetration=0.0,
            name="negative lock knob seats on boss",
        )

        rest_aabb = ctx.part_element_world_aabb(can, elem="front_lens")

    def _z_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[2] + hi[2]) / 2.0

    with ctx.pose({tilt: -0.60}):
        up_aabb = ctx.part_element_world_aabb(can, elem="front_lens")
    with ctx.pose({tilt: 0.60}):
        down_aabb = ctx.part_element_world_aabb(can, elem="front_lens")

    rest_z = _z_center(rest_aabb)
    up_z = _z_center(up_aabb)
    down_z = _z_center(down_aabb)
    ctx.check(
        "tilt aiming moves the beam through repeatable limits",
        rest_z is not None
        and up_z is not None
        and down_z is not None
        and up_z > rest_z + 0.08
        and down_z < rest_z - 0.08,
        details=f"rest_z={rest_z}, up_z={up_z}, down_z={down_z}",
    )

    return ctx.report()


object_model = build_object_model()
