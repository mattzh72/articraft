from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _tube_mesh(
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    name: str,
    samples_per_segment: int = 12,
    radial_segments: int = 18,
) -> object:
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
        ),
        name,
    )


def _add_tube(
    part,
    *,
    name: str,
    points: list[tuple[float, float, float]],
    radius: float,
    material,
    samples_per_segment: int = 12,
    radial_segments: int = 18,
) -> None:
    part.visual(
        _tube_mesh(
            points,
            radius=radius,
            name=name,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
        ),
        material=material,
        name=name,
    )


def _build_wheel(part, *, inner_sign: float, rubber, rim_metal, hub_metal) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=0.18, length=0.07),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.128, length=0.054),
        origin=spin_origin,
        material=rim_metal,
        name="rim_shell",
    )
    part.visual(
        Cylinder(radius=0.108, length=0.010),
        origin=Origin(xyz=(-0.022, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_metal,
        name="inner_rim_disc",
    )
    part.visual(
        Cylinder(radius=0.108, length=0.010),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_metal,
        name="outer_rim_disc",
    )
    part.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(inner_sign * 0.022, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_metal,
        name="inner_hub",
    )
    part.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(-inner_sign * 0.022, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_metal,
        name="outer_hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="keg_hand_truck")

    frame_blue = model.material("frame_blue", rgba=(0.13, 0.22, 0.34, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.65, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    hook_red = model.material("hook_red", rgba=(0.72, 0.14, 0.12, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.58, 0.40, 1.40)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.01, 0.70)),
    )

    left_rail_points = [
        (0.130, 0.020, 0.070),
        (0.148, -0.010, 0.340),
        (0.160, -0.040, 0.740),
        (0.148, -0.080, 1.090),
        (0.110, -0.110, 1.350),
    ]
    _add_tube(
        frame,
        name="left_main_rail",
        points=left_rail_points,
        radius=0.022,
        material=frame_blue,
        samples_per_segment=18,
    )
    _add_tube(
        frame,
        name="right_main_rail",
        points=_mirror_x(left_rail_points),
        radius=0.022,
        material=frame_blue,
        samples_per_segment=18,
    )
    _add_tube(
        frame,
        name="lower_crossbar",
        points=[(-0.130, 0.020, 0.070), (0.130, 0.020, 0.070)],
        radius=0.018,
        material=frame_blue,
        samples_per_segment=2,
    )
    _add_tube(
        frame,
        name="mid_crossbar_low",
        points=[(-0.153, -0.025, 0.500), (0.153, -0.025, 0.500)],
        radius=0.016,
        material=frame_blue,
        samples_per_segment=2,
    )
    _add_tube(
        frame,
        name="mid_crossbar_high",
        points=[(-0.155, -0.055, 0.860), (0.155, -0.055, 0.860)],
        radius=0.016,
        material=frame_blue,
        samples_per_segment=2,
    )
    _add_tube(
        frame,
        name="shoulder_crossbar",
        points=[(-0.138, -0.090, 1.170), (0.138, -0.090, 1.170)],
        radius=0.017,
        material=frame_blue,
        samples_per_segment=2,
    )
    _add_tube(
        frame,
        name="top_handle",
        points=[(-0.110, -0.135, 1.345), (0.110, -0.135, 1.345)],
        radius=0.020,
        material=dark_steel,
        samples_per_segment=2,
    )
    frame.visual(
        Box((0.340, 0.220, 0.012)),
        origin=Origin(xyz=(0.0, 0.135, 0.006)),
        material=steel,
        name="toe_plate",
    )
    frame.visual(
        Box((0.340, 0.020, 0.085)),
        origin=Origin(xyz=(0.0, 0.245, 0.048)),
        material=steel,
        name="toe_lip",
    )
    frame.visual(
        Box((0.045, 0.140, 0.080)),
        origin=Origin(xyz=(0.115, 0.075, 0.040)),
        material=frame_blue,
        name="left_toe_support",
    )
    frame.visual(
        Box((0.045, 0.140, 0.080)),
        origin=Origin(xyz=(-0.115, 0.075, 0.040)),
        material=frame_blue,
        name="right_toe_support",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.440),
        origin=Origin(xyz=(0.0, -0.100, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_beam",
    )
    frame.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.217, -0.100, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_spacer",
    )
    frame.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(-0.217, -0.100, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_spacer",
    )
    _add_tube(
        frame,
        name="left_lower_axle_brace",
        points=[(0.127, 0.015, 0.105), (0.196, -0.100, 0.180)],
        radius=0.015,
        material=frame_blue,
        samples_per_segment=2,
    )
    _add_tube(
        frame,
        name="right_lower_axle_brace",
        points=[(-0.127, 0.015, 0.105), (-0.196, -0.100, 0.180)],
        radius=0.015,
        material=frame_blue,
        samples_per_segment=2,
    )
    _add_tube(
        frame,
        name="left_upper_axle_brace",
        points=[(0.145, -0.005, 0.355), (0.188, -0.100, 0.220)],
        radius=0.014,
        material=frame_blue,
        samples_per_segment=2,
    )
    _add_tube(
        frame,
        name="right_upper_axle_brace",
        points=[(-0.145, -0.005, 0.355), (-0.188, -0.100, 0.220)],
        radius=0.014,
        material=frame_blue,
        samples_per_segment=2,
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(0.061, -0.090, 1.325), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_hinge_ear",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(-0.061, -0.090, 1.325), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_hinge_ear",
    )
    frame.visual(
        Box((0.018, 0.040, 0.070)),
        origin=Origin(xyz=(0.061, -0.110, 1.325)),
        material=steel,
        name="right_hinge_bracket",
    )
    frame.visual(
        Box((0.018, 0.040, 0.070)),
        origin=Origin(xyz=(-0.061, -0.110, 1.325)),
        material=steel,
        name="left_hinge_bracket",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.07),
        mass=3.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_wheel(
        left_wheel,
        inner_sign=-1.0,
        rubber=rubber,
        rim_metal=steel,
        hub_metal=dark_steel,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.07),
        mass=3.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_wheel(
        right_wheel,
        inner_sign=1.0,
        rubber=rubber,
        rim_metal=steel,
        hub_metal=dark_steel,
    )

    retaining_hook = model.part("retaining_hook")
    retaining_hook.inertial = Inertial.from_geometry(
        Box((0.14, 0.10, 0.24)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.015, 0.110)),
    )
    retaining_hook.visual(
        Cylinder(radius=0.013, length=0.092),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_sleeve",
    )
    _add_tube(
        retaining_hook,
        name="left_hook_arm",
        points=[
            (-0.010, 0.000, 0.000),
            (-0.028, -0.010, 0.040),
            (-0.052, -0.020, 0.100),
            (-0.058, -0.030, 0.160),
        ],
        radius=0.011,
        material=hook_red,
        samples_per_segment=10,
        radial_segments=16,
    )
    _add_tube(
        retaining_hook,
        name="right_hook_arm",
        points=[
            (0.010, 0.000, 0.000),
            (0.028, -0.010, 0.040),
            (0.052, -0.020, 0.100),
            (0.058, -0.030, 0.160),
        ],
        radius=0.011,
        material=hook_red,
        samples_per_segment=10,
        radial_segments=16,
    )
    retaining_hook.visual(
        Cylinder(radius=0.012, length=0.116),
        origin=Origin(xyz=(0.0, -0.030, 0.160), rpy=(0.0, pi / 2.0, 0.0)),
        material=hook_red,
        name="hook_bar",
    )
    _add_tube(
        retaining_hook,
        name="hook_finger",
        points=[(0.0, -0.030, 0.160), (0.0, -0.010, 0.210), (0.0, 0.016, 0.228)],
        radius=0.010,
        material=hook_red,
        samples_per_segment=8,
        radial_segments=14,
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(0.255, -0.100, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(-0.255, -0.100, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "hook_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=retaining_hook,
        origin=Origin(xyz=(0.0, -0.090, 1.325)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    retaining_hook = object_model.get_part("retaining_hook")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")
    hook_hinge = object_model.get_articulation("hook_hinge")

    ctx.check(
        "all major parts are present",
        all(part is not None for part in (frame, left_wheel, right_wheel, retaining_hook)),
        details="Expected frame, two rear wheels, and retaining hook.",
    )
    ctx.check(
        "rear wheel joints spin on the axle line",
        left_wheel_spin.axis == (1.0, 0.0, 0.0) and right_wheel_spin.axis == (1.0, 0.0, 0.0),
        details=f"left_axis={left_wheel_spin.axis}, right_axis={right_wheel_spin.axis}",
    )
    ctx.check(
        "retaining hook uses a transverse hinge",
        hook_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"hook_axis={hook_hinge.axis}",
    )

    ctx.expect_contact(
        left_wheel,
        frame,
        elem_a="inner_hub",
        elem_b="left_spacer",
        contact_tol=0.0015,
        name="left wheel mounts against the axle spacer",
    )
    ctx.expect_contact(
        right_wheel,
        frame,
        elem_a="inner_hub",
        elem_b="right_spacer",
        contact_tol=0.0015,
        name="right wheel mounts against the axle spacer",
    )
    ctx.expect_contact(
        retaining_hook,
        frame,
        elem_a="hinge_sleeve",
        elem_b="left_hinge_ear",
        contact_tol=0.0005,
        name="retaining hook sleeve bears on the left hinge ear",
    )
    ctx.expect_contact(
        retaining_hook,
        frame,
        elem_a="hinge_sleeve",
        elem_b="right_hinge_ear",
        contact_tol=0.0005,
        name="retaining hook sleeve bears on the right hinge ear",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    rest_bar = ctx.part_element_world_aabb(retaining_hook, elem="hook_bar")
    rest_center = _aabb_center(rest_bar)
    upper_limit = hook_hinge.motion_limits.upper if hook_hinge.motion_limits is not None else None
    with ctx.pose({hook_hinge: upper_limit if upper_limit is not None else 2.0}):
        folded_bar = ctx.part_element_world_aabb(retaining_hook, elem="hook_bar")
    folded_center = _aabb_center(folded_bar)
    ctx.check(
        "hook folds forward and down over the load path",
        rest_center is not None
        and folded_center is not None
        and folded_center[1] > rest_center[1] + 0.10
        and folded_center[2] < rest_center[2] - 0.12,
        details=f"rest_center={rest_center}, folded_center={folded_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
