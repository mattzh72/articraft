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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_wheel_visuals(part, mesh_prefix: str, *, rubber, rim_metal, hub_metal) -> None:
    part.visual(
        _save_mesh(
            f"{mesh_prefix}_tire",
            TorusGeometry(radius=0.085, tube=0.035, radial_segments=18, tubular_segments=36),
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.072, length=0.050),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rim_metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hub_metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rim_metal,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adult_mobility_scooter")

    body_blue = model.material("body_blue", rgba=(0.20, 0.37, 0.60, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.13, 0.14, 0.15, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.15, 0.15, 0.16, 1.0))
    control_black = model.material("control_black", rgba=(0.09, 0.10, 0.11, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.82, 0.54, 0.080)),
        origin=Origin(xyz=(0.00, 0.00, 0.135)),
        material=body_blue,
        name="deck_main",
    )
    chassis.visual(
        Box((0.22, 0.44, 0.070)),
        origin=Origin(xyz=(0.49, 0.00, 0.130)),
        material=body_blue,
        name="deck_nose",
    )
    chassis.visual(
        Box((0.16, 0.40, 0.010)),
        origin=Origin(xyz=(-0.24, 0.00, 0.180)),
        material=dark_trim,
        name="rear_deck_pad",
    )
    chassis.visual(
        Box((0.38, 0.40, 0.010)),
        origin=Origin(xyz=(0.17, 0.00, 0.180)),
        material=dark_trim,
        name="front_deck_pad",
    )
    chassis.visual(
        Box((0.12, 0.055, 0.030)),
        origin=Origin(xyz=(-0.10, 0.0625, 0.190)),
        material=dark_trim,
        name="left_seat_base_plinth",
    )
    chassis.visual(
        Box((0.12, 0.055, 0.030)),
        origin=Origin(xyz=(-0.10, -0.0625, 0.190)),
        material=dark_trim,
        name="right_seat_base_plinth",
    )

    seat_tube_mesh = _save_mesh(
        "seat_tube_shell",
        ExtrudeWithHolesGeometry(
            superellipse_profile(0.086, 0.086, exponent=2.0, segments=48),
            [superellipse_profile(0.072, 0.072, exponent=2.0, segments=48)],
            height=0.160,
            center=True,
        ),
    )
    chassis.visual(
        seat_tube_mesh,
        origin=Origin(xyz=(-0.10, 0.00, 0.255)),
        material=dark_steel,
        name="seat_tube",
    )
    chassis.visual(
        Box((0.070, 0.028, 0.028)),
        origin=Origin(xyz=(-0.10, 0.050, 0.189)),
        material=dark_steel,
        name="seat_collar_left",
    )
    chassis.visual(
        Box((0.070, 0.028, 0.028)),
        origin=Origin(xyz=(-0.10, -0.050, 0.189)),
        material=dark_steel,
        name="seat_collar_right",
    )
    chassis.visual(
        Box((0.11, 0.16, 0.100)),
        origin=Origin(xyz=(0.35, 0.00, 0.225)),
        material=body_blue,
        name="neck_block",
    )
    chassis.visual(
        Cylinder(radius=0.045, length=0.100),
        origin=Origin(xyz=(0.38, 0.00, 0.275)),
        material=dark_steel,
        name="steering_socket",
    )
    chassis.visual(
        Box((0.16, 0.05, 0.060)),
        origin=Origin(xyz=(0.39, 0.245, 0.120)),
        material=body_blue,
        name="left_front_outrigger",
    )
    chassis.visual(
        Box((0.16, 0.05, 0.060)),
        origin=Origin(xyz=(0.39, -0.245, 0.120)),
        material=body_blue,
        name="right_front_outrigger",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((1.10, 0.66, 0.38)),
        mass=68.0,
        origin=Origin(xyz=(0.02, 0.00, 0.165)),
    )

    seat_assembly = model.part("seat_assembly")
    seat_assembly.visual(
        Cylinder(radius=0.032, length=0.400),
        origin=Origin(xyz=(0.00, 0.00, 0.040)),
        material=steel,
        name="seat_post",
    )
    seat_assembly.visual(
        Box((0.12, 0.12, 0.020)),
        origin=Origin(xyz=(0.00, 0.00, 0.135)),
        material=dark_steel,
        name="seat_mount_plate",
    )
    seat_assembly.visual(
        Box((0.42, 0.37, 0.060)),
        origin=Origin(xyz=(0.03, 0.00, 0.180)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat_assembly.visual(
        Box((0.10, 0.34, 0.340)),
        origin=Origin(xyz=(-0.13, 0.00, 0.355)),
        material=seat_vinyl,
        name="backrest",
    )
    seat_assembly.inertial = Inertial.from_geometry(
        Box((0.44, 0.38, 0.72)),
        mass=11.0,
        origin=Origin(xyz=(0.00, 0.00, 0.240)),
    )

    tiller = model.part("tiller")
    tiller.visual(
        Box((0.08, 0.10, 0.040)),
        origin=Origin(xyz=(0.00, 0.00, 0.020)),
        material=dark_steel,
        name="hinge_block",
    )
    tiller.visual(
        Cylinder(radius=0.024, length=0.500),
        origin=Origin(xyz=(0.00, 0.00, 0.250), rpy=(0.0, -0.18, 0.0)),
        material=steel,
        name="stem_tube",
    )
    tiller.visual(
        Box((0.13, 0.08, 0.065)),
        origin=Origin(xyz=(-0.09, 0.00, 0.435), rpy=(0.0, -0.10, 0.0)),
        material=control_black,
        name="dashboard",
    )
    tiller.visual(
        Box((0.060, 0.070, 0.120)),
        origin=Origin(xyz=(-0.115, 0.000, 0.485)),
        material=dark_steel,
        name="handlebar_riser",
    )
    tiller.visual(
        _save_mesh(
            "tiller_handlebar_loop",
            tube_from_spline_points(
                [
                    (-0.03, -0.23, 0.44),
                    (-0.08, -0.15, 0.49),
                    (-0.12, -0.06, 0.52),
                    (-0.14, 0.00, 0.53),
                    (-0.12, 0.06, 0.52),
                    (-0.08, 0.15, 0.49),
                    (-0.03, 0.23, 0.44),
                ],
                radius=0.016,
                samples_per_segment=12,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=dark_steel,
        name="handlebar",
    )
    tiller.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(-0.03, 0.212, 0.440), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    tiller.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(-0.03, -0.212, 0.440), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    tiller.inertial = Inertial.from_geometry(
        Box((0.32, 0.56, 0.60)),
        mass=7.5,
        origin=Origin(xyz=(-0.05, 0.00, 0.300)),
    )

    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        _add_wheel_visuals(
            wheel,
            wheel_name,
            rubber=rubber,
            rim_metal=dark_steel,
            hub_metal=steel,
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.120, length=0.070),
            mass=4.8,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        )

    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=seat_assembly,
        origin=Origin(xyz=(-0.10, 0.00, 0.335)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.08,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "tiller_fold",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=tiller,
        origin=Origin(xyz=(0.38, 0.00, 0.325)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=-0.95,
            upper=0.10,
        ),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child="front_left_wheel",
        origin=Origin(xyz=(0.392, 0.305, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child="front_right_wheel",
        origin=Origin(xyz=(0.392, -0.305, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child="rear_left_wheel",
        origin=Origin(xyz=(-0.330, 0.305, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=18.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child="rear_right_wheel",
        origin=Origin(xyz=(-0.330, -0.305, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=18.0),
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

    chassis = object_model.get_part("chassis")
    seat_assembly = object_model.get_part("seat_assembly")
    tiller = object_model.get_part("tiller")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    seat_height = object_model.get_articulation("seat_height")
    tiller_fold = object_model.get_articulation("tiller_fold")
    front_left_wheel_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_wheel_spin = object_model.get_articulation("front_right_wheel_spin")
    rear_left_wheel_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_wheel_spin = object_model.get_articulation("rear_right_wheel_spin")

    ctx.check(
        "seat post uses vertical prismatic travel",
        seat_height.articulation_type == ArticulationType.PRISMATIC
        and seat_height.axis == (0.0, 0.0, 1.0)
        and seat_height.motion_limits is not None
        and seat_height.motion_limits.lower == 0.0
        and seat_height.motion_limits.upper is not None
        and seat_height.motion_limits.upper >= 0.07,
        details=str(
            {
                "type": seat_height.articulation_type,
                "axis": seat_height.axis,
                "limits": seat_height.motion_limits,
            }
        ),
    )
    ctx.check(
        "tiller hinge uses fore-aft revolute fold",
        tiller_fold.articulation_type == ArticulationType.REVOLUTE
        and tiller_fold.axis == (0.0, 1.0, 0.0)
        and tiller_fold.motion_limits is not None
        and tiller_fold.motion_limits.lower is not None
        and tiller_fold.motion_limits.lower <= -0.9,
        details=str(
            {
                "type": tiller_fold.articulation_type,
                "axis": tiller_fold.axis,
                "limits": tiller_fold.motion_limits,
            }
        ),
    )

    for name, joint in (
        ("front left wheel spins continuously", front_left_wheel_spin),
        ("front right wheel spins continuously", front_right_wheel_spin),
        ("rear left wheel spins continuously", rear_left_wheel_spin),
        ("rear right wheel spins continuously", rear_right_wheel_spin),
    ):
        limits = joint.motion_limits
        ctx.check(
            name,
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=str({"type": joint.articulation_type, "axis": joint.axis, "limits": limits}),
        )

    with ctx.pose({seat_height: 0.0}):
        ctx.expect_within(
            seat_assembly,
            chassis,
            axes="xy",
            inner_elem="seat_post",
            outer_elem="seat_tube",
            name="collapsed seat post stays centered in sleeve",
        )
        ctx.expect_overlap(
            seat_assembly,
            chassis,
            axes="z",
            elem_a="seat_post",
            elem_b="seat_tube",
            min_overlap=0.15,
            name="collapsed seat post remains deeply inserted",
        )

    with ctx.pose({seat_height: seat_height.motion_limits.upper}):
        ctx.expect_within(
            seat_assembly,
            chassis,
            axes="xy",
            inner_elem="seat_post",
            outer_elem="seat_tube",
            name="raised seat post stays centered in sleeve",
        )
        ctx.expect_overlap(
            seat_assembly,
            chassis,
            axes="z",
            elem_a="seat_post",
            elem_b="seat_tube",
            min_overlap=0.07,
            name="raised seat post retains insertion",
        )

    ctx.expect_contact(
        rear_left_wheel,
        chassis,
        elem_a="hub",
        elem_b="deck_main",
        contact_tol=5e-4,
        name="rear left wheel is mounted against the deck side",
    )
    ctx.expect_contact(
        rear_right_wheel,
        chassis,
        elem_a="hub",
        elem_b="deck_main",
        contact_tol=5e-4,
        name="rear right wheel is mounted against the deck side",
    )
    ctx.expect_contact(
        front_left_wheel,
        chassis,
        elem_a="hub",
        elem_b="left_front_outrigger",
        contact_tol=5e-4,
        name="front left wheel is mounted on the left outrigger",
    )
    ctx.expect_contact(
        front_right_wheel,
        chassis,
        elem_a="hub",
        elem_b="right_front_outrigger",
        contact_tol=5e-4,
        name="front right wheel is mounted on the right outrigger",
    )

    dashboard_rest = ctx.part_element_world_aabb(tiller, elem="dashboard")
    with ctx.pose({tiller_fold: tiller_fold.motion_limits.lower}):
        dashboard_folded = ctx.part_element_world_aabb(tiller, elem="dashboard")
        ctx.expect_gap(
            tiller,
            chassis,
            axis="z",
            positive_elem="dashboard",
            negative_elem="deck_main",
            max_penetration=0.0,
            name="folded tiller dashboard stays above the deck",
        )

    rest_center = None
    folded_center = None
    if dashboard_rest is not None:
        rest_center = (
            0.5 * (dashboard_rest[0][0] + dashboard_rest[1][0]),
            0.5 * (dashboard_rest[0][1] + dashboard_rest[1][1]),
            0.5 * (dashboard_rest[0][2] + dashboard_rest[1][2]),
        )
    if dashboard_folded is not None:
        folded_center = (
            0.5 * (dashboard_folded[0][0] + dashboard_folded[1][0]),
            0.5 * (dashboard_folded[0][1] + dashboard_folded[1][1]),
            0.5 * (dashboard_folded[0][2] + dashboard_folded[1][2]),
        )
    ctx.check(
        "folded tiller lowers the controls toward the rider",
        rest_center is not None
        and folded_center is not None
        and folded_center[2] < rest_center[2] - 0.10
        and folded_center[0] < rest_center[0] - 0.12,
        details=str({"rest_center": rest_center, "folded_center": folded_center}),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
