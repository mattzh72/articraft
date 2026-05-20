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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mn, mx = aabb
    return tuple((mn[i] + mx[i]) * 0.5 for i in range(3))


def _bell_shell_mesh():
    profile = [
        (0.0, -0.018),
        (0.112, -0.018),
        (0.155, -0.040),
        (0.235, -0.090),
        (0.330, -0.180),
        (0.405, -0.360),
        (0.432, -0.640),
        (0.445, -0.980),
        (0.470, -1.220),
        (0.490, -1.285),
        (0.430, -1.248),
        (0.395, -1.030),
        (0.372, -0.710),
        (0.346, -0.360),
        (0.278, -0.185),
        (0.188, -0.080),
        (0.090, -0.060),
        (0.0, -0.060),
    ]
    return LatheGeometry(profile, segments=88)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bonsho_bell_frame")

    timber = model.material("timber", rgba=(0.46, 0.32, 0.18, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.34, 0.22, 0.12, 1.0))
    aged_bronze = model.material("aged_bronze", rgba=(0.50, 0.37, 0.19, 1.0))
    iron = model.material("iron", rgba=(0.22, 0.23, 0.25, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((2.50, 0.52, 2.75)),
        mass=620.0,
        origin=Origin(xyz=(0.0, 0.0, 1.375)),
    )
    frame.visual(
        Box((0.34, 0.40, 0.12)),
        origin=Origin(xyz=(-0.88, 0.0, 0.06)),
        material=timber,
        name="left_foot",
    )
    frame.visual(
        Box((0.34, 0.40, 0.12)),
        origin=Origin(xyz=(0.88, 0.0, 0.06)),
        material=timber,
        name="right_foot",
    )
    frame.visual(
        Box((0.22, 0.24, 2.52)),
        origin=Origin(xyz=(-0.88, 0.0, 1.26)),
        material=timber,
        name="left_post",
    )
    frame.visual(
        Box((0.22, 0.24, 2.52)),
        origin=Origin(xyz=(0.88, 0.0, 1.26)),
        material=timber,
        name="right_post",
    )
    frame.visual(
        Box((2.08, 0.30, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 2.56)),
        material=timber,
        name="crossbeam",
    )
    frame.visual(
        Box((1.92, 0.22, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=timber,
        name="lower_tie",
    )
    frame.visual(
        Box((0.14, 0.22, 0.50)),
        origin=Origin(xyz=(-0.80, 0.20, 2.25), rpy=(0.0, 0.62, 0.0)),
        material=dark_timber,
        name="left_brace",
    )
    frame.visual(
        Box((0.14, 0.22, 0.50)),
        origin=Origin(xyz=(0.80, -0.20, 2.25), rpy=(0.0, -0.62, 0.0)),
        material=dark_timber,
        name="right_brace",
    )
    frame.visual(
        Box((0.18, 0.18, 0.34)),
        origin=Origin(xyz=(-0.78, 0.0, 2.14)),
        material=dark_timber,
        name="pivot_arm",
    )
    frame.visual(
        Box((0.05, 0.05, 0.18)),
        origin=Origin(xyz=(-0.70, 0.055, 2.13)),
        material=iron,
        name="pivot_cheek_front",
    )
    frame.visual(
        Box((0.05, 0.05, 0.18)),
        origin=Origin(xyz=(-0.70, -0.055, 2.13)),
        material=iron,
        name="pivot_cheek_back",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.16),
        origin=Origin(xyz=(-0.70, 0.0, 2.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="pivot_pin",
    )
    frame.visual(
        Box((0.03, 0.03, 0.10)),
        origin=Origin(xyz=(-0.09, 0.09, 2.43)),
        material=iron,
        name="bell_hanger_front",
    )
    frame.visual(
        Box((0.03, 0.03, 0.10)),
        origin=Origin(xyz=(-0.09, -0.09, 2.43)),
        material=iron,
        name="bell_hanger_back",
    )
    frame.visual(
        Box((0.03, 0.03, 0.10)),
        origin=Origin(xyz=(0.09, 0.09, 2.43)),
        material=iron,
        name="bell_hanger_front_right",
    )
    frame.visual(
        Box((0.03, 0.03, 0.10)),
        origin=Origin(xyz=(0.09, -0.09, 2.43)),
        material=iron,
        name="bell_hanger_back_right",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 2.43), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="bell_crown_pin",
    )

    bell = model.part("bell")
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.50, length=1.32),
        mass=290.0,
        origin=Origin(xyz=(0.0, 0.0, -0.66)),
    )
    bell.visual(
        _save_mesh("bonsho_shell", _bell_shell_mesh()),
        material=aged_bronze,
        name="bell_shell",
    )
    bell.visual(
        Box((0.18, 0.15, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=aged_bronze,
        name="bell_crown_cap",
    )
    bell.visual(
        Box((0.05, 0.10, 0.12)),
        origin=Origin(xyz=(-0.06, 0.0, -0.070)),
        material=aged_bronze,
        name="bell_crown_left",
    )
    bell.visual(
        Box((0.05, 0.10, 0.12)),
        origin=Origin(xyz=(0.06, 0.0, -0.070)),
        material=aged_bronze,
        name="bell_crown_right",
    )
    bell.visual(
        Cylinder(radius=0.10, length=0.06),
        origin=Origin(xyz=(-0.36, 0.0, -0.78), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_bronze,
        name="strike_boss",
    )

    striker = model.part("striker")
    striker.inertial = Inertial.from_geometry(
        Box((0.26, 1.00, 0.78)),
        mass=75.0,
        origin=Origin(xyz=(0.04, 0.0, -0.35)),
    )
    striker.visual(
        Box((0.06, 0.035, 0.46)),
        origin=Origin(xyz=(0.04, 0.0, -0.245)),
        material=dark_timber,
        name="striker_hanger",
    )
    striker.visual(
        Box((0.08, 0.18, 0.07)),
        origin=Origin(xyz=(0.04, 0.0, -0.508)),
        material=iron,
        name="striker_yoke",
    )
    striker.visual(
        Cylinder(radius=0.095, length=0.94),
        origin=Origin(xyz=(0.04, 0.0, -0.60), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_timber,
        name="striker_log",
    )
    striker.visual(
        Cylinder(radius=0.060, length=0.10),
        origin=Origin(xyz=(0.04, 0.0, -0.60), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="striker_band",
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 2.39)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.7,
            lower=-0.18,
            upper=0.18,
        ),
    )
    model.articulation(
        "striker_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=striker,
        origin=Origin(xyz=(-0.70, 0.0, 2.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=1.0,
            lower=0.0,
            upper=0.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    bell = object_model.get_part("bell")
    striker = object_model.get_part("striker")
    bell_swing = object_model.get_articulation("bell_swing")
    striker_swing = object_model.get_articulation("striker_swing")

    ctx.check(
        "bell swing axis and limits are plausible",
        bell_swing.axis == (0.0, 1.0, 0.0)
        and bell_swing.motion_limits is not None
        and bell_swing.motion_limits.lower is not None
        and bell_swing.motion_limits.upper is not None
        and bell_swing.motion_limits.lower < 0.0 < bell_swing.motion_limits.upper,
        details=f"axis={bell_swing.axis}, limits={bell_swing.motion_limits}",
    )
    ctx.check(
        "striker swings inward from the post",
        striker_swing.axis == (0.0, -1.0, 0.0)
        and striker_swing.motion_limits is not None
        and striker_swing.motion_limits.lower == 0.0
        and striker_swing.motion_limits.upper is not None
        and striker_swing.motion_limits.upper >= 0.25,
        details=f"axis={striker_swing.axis}, limits={striker_swing.motion_limits}",
    )
    ctx.expect_origin_distance(
        bell,
        frame,
        axes="x",
        max_dist=0.01,
        name="bell stays centered under the frame",
    )

    with ctx.pose({bell_swing: 0.0, striker_swing: 0.0}):
        ctx.expect_contact(
            frame,
            bell,
            elem_a="bell_hanger_front",
            elem_b="bell_crown_cap",
            contact_tol=0.001,
            name="bell crown cap bears on the front hanger bracket",
        )
        ctx.expect_gap(
            frame,
            bell,
            axis="z",
            positive_elem="crossbeam",
            negative_elem="bell_crown_cap",
            min_gap=0.02,
            max_gap=0.10,
            name="bell crown hangs just below the crossbeam",
        )
        ctx.expect_gap(
            bell,
            striker,
            axis="x",
            positive_elem="bell_shell",
            negative_elem="striker_log",
            min_gap=0.03,
            name="resting striker clears the bell",
        )
        ctx.expect_contact(
            frame,
            striker,
            elem_a="pivot_pin",
            elem_b="striker_hanger",
            contact_tol=0.002,
            name="striker hanger sits on the pivot pin",
        )

        rest_log_center = _aabb_center(ctx.part_element_world_aabb(striker, elem="striker_log"))

    with ctx.pose({striker_swing: 0.32}):
        ctx.expect_contact(
            bell,
            striker,
            elem_a="strike_boss",
            elem_b="striker_log",
            contact_tol=0.03,
            name="swung striker reaches the bell strike boss",
        )
        swung_log_center = _aabb_center(ctx.part_element_world_aabb(striker, elem="striker_log"))

    ctx.check(
        "striker log moves toward the bell when swung",
        rest_log_center is not None
        and swung_log_center is not None
        and swung_log_center[0] > rest_log_center[0] + 0.08,
        details=f"rest={rest_log_center}, swung={swung_log_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
