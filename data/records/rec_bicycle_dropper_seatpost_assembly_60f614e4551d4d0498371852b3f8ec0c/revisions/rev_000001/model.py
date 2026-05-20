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
    Sphere,
    surface_frame,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _shell_tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 56,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, length)],
            [(inner_radius, 0.0), (inner_radius, length)],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electronic_dropper_seatpost")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.17, 0.17, 0.18, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    alloy = model.material("alloy", rgba=(0.64, 0.66, 0.69, 1.0))
    anodized = model.material("anodized", rgba=(0.30, 0.32, 0.35, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    bike_stub = model.part("bike_stub")
    bike_stub.visual(
        _shell_tube_mesh(
            "frame_seat_tube_shell",
            outer_radius=0.0180,
            inner_radius=0.0162,
            length=0.120,
        ),
        material=matte_black,
        name="seat_tube",
    )
    bike_stub.visual(
        Box((0.0006, 0.0050, 0.020)),
        origin=Origin(xyz=(0.0159, 0.0000, 0.096)),
        material=anodized,
        name="upper_post_guide",
    )
    bike_stub.visual(
        Box((0.0006, 0.0050, 0.020)),
        origin=Origin(xyz=(-0.0159, 0.0000, 0.096)),
        material=anodized,
        name="upper_post_guide_rear",
    )
    bike_stub.visual(
        Box((0.0050, 0.0006, 0.020)),
        origin=Origin(xyz=(0.0000, 0.0159, 0.096)),
        material=anodized,
        name="upper_post_guide_left",
    )
    bike_stub.visual(
        Box((0.0050, 0.0006, 0.020)),
        origin=Origin(xyz=(0.0000, -0.0159, 0.096)),
        material=anodized,
        name="upper_post_guide_right",
    )
    bike_stub.visual(
        Box((0.0006, 0.0050, 0.016)),
        origin=Origin(xyz=(0.0159, 0.0000, 0.018)),
        material=anodized,
        name="lower_post_guide",
    )
    bike_stub.visual(
        Box((0.0006, 0.0050, 0.016)),
        origin=Origin(xyz=(-0.0159, 0.0000, 0.018)),
        material=anodized,
        name="lower_post_guide_rear",
    )
    bike_stub.visual(
        Box((0.0050, 0.0006, 0.016)),
        origin=Origin(xyz=(0.0000, 0.0159, 0.018)),
        material=anodized,
        name="lower_post_guide_left",
    )
    bike_stub.visual(
        Box((0.0050, 0.0006, 0.016)),
        origin=Origin(xyz=(0.0000, -0.0159, 0.018)),
        material=anodized,
        name="lower_post_guide_right",
    )
    bike_stub.visual(
        _shell_tube_mesh(
            "frame_seat_collar",
            outer_radius=0.0205,
            inner_radius=0.0180,
            length=0.018,
        ),
        origin=Origin(xyz=(0.000, 0.000, 0.102)),
        material=dark_gray,
        name="seat_collar",
    )
    bike_stub.visual(
        Cylinder(radius=0.0028, length=0.030),
        origin=Origin(xyz=(-0.026, 0.000, 0.111), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="seat_collar_bolt",
    )
    bike_stub.visual(
        Box((0.009, 0.010, 0.018)),
        origin=Origin(xyz=(-0.0235, 0.000, 0.111)),
        material=dark_gray,
        name="seat_collar_jaw",
    )
    bike_stub.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.034, 0.000, 0.104), (0.195, 0.004, 0.134), (0.430, 0.000, 0.230)],
                radius=0.0140,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
            "frame_top_tube",
        ),
        material=matte_black,
        name="top_tube",
    )
    bike_stub.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.036, 0.000, 0.060), (0.200, -0.004, 0.108), (0.425, 0.000, 0.138)],
                radius=0.0155,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
            "frame_down_tube",
        ),
        material=matte_black,
        name="down_tube",
    )
    bike_stub.visual(
        Box((0.020, 0.026, 0.032)),
        origin=Origin(xyz=(0.0262, 0.000, 0.104)),
        material=matte_black,
        name="seat_cluster_upper",
    )
    bike_stub.visual(
        Box((0.024, 0.028, 0.048)),
        origin=Origin(xyz=(0.0282, 0.000, 0.066)),
        material=matte_black,
        name="seat_cluster_lower",
    )
    bike_stub.visual(
        Cylinder(radius=0.0230, length=0.160),
        origin=Origin(xyz=(0.440, 0.000, 0.180)),
        material=matte_black,
        name="head_tube",
    )
    bike_stub.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.440, 0.000, 0.248), (0.475, 0.000, 0.272), (0.525, 0.000, 0.314)],
                radius=0.0140,
                samples_per_segment=12,
                radial_segments=18,
                cap_ends=True,
            ),
            "cockpit_stem",
        ),
        material=dark_gray,
        name="stem",
    )
    handlebar_visual = bike_stub.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.555, -0.290, 0.300),
                    (0.544, -0.190, 0.309),
                    (0.531, -0.085, 0.321),
                    (0.525, 0.000, 0.325),
                    (0.531, 0.085, 0.321),
                    (0.544, 0.190, 0.309),
                    (0.555, 0.290, 0.300),
                ],
                radius=0.0110,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
            "cockpit_handlebar",
        ),
        material=dark_gray,
        name="handlebar",
    )
    bike_stub.visual(
        Cylinder(radius=0.0165, length=0.110),
        origin=Origin(xyz=(0.555, -0.345, 0.300), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    bike_stub.visual(
        Cylinder(radius=0.0165, length=0.110),
        origin=Origin(xyz=(0.555, 0.345, 0.300), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    bike_stub.inertial = Inertial.from_geometry(
        Box((0.690, 0.690, 0.360)),
        mass=2.0,
        origin=Origin(xyz=(0.275, 0.000, 0.180)),
    )

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        _shell_tube_mesh(
            "seatpost_outer_sleeve",
            outer_radius=0.0156,
            inner_radius=0.0138,
            length=0.285,
        ),
        material=satin_black,
        name="outer_sleeve",
    )
    outer_tube.visual(
        _shell_tube_mesh(
            "seatpost_head_collar",
            outer_radius=0.0182,
            inner_radius=0.0135,
            length=0.035,
        ),
        origin=Origin(xyz=(0.000, 0.000, 0.265)),
        material=dark_gray,
        name="head_collar",
    )
    outer_tube.visual(
        Box((0.0014, 0.006, 0.022)),
        origin=Origin(xyz=(0.0138, 0.000, 0.289)),
        material=anodized,
        name="guide_pad_front",
    )
    outer_tube.visual(
        Box((0.0014, 0.006, 0.022)),
        origin=Origin(xyz=(-0.0138, 0.000, 0.289)),
        material=anodized,
        name="guide_pad_rear",
    )
    outer_tube.visual(
        Box((0.006, 0.0014, 0.022)),
        origin=Origin(xyz=(0.000, 0.0138, 0.289)),
        material=anodized,
        name="guide_pad_left",
    )
    outer_tube.visual(
        Box((0.010, 0.012, 0.034)),
        origin=Origin(xyz=(-0.020, 0.000, 0.236)),
        material=dark_gray,
        name="battery_bridge",
    )
    outer_tube.visual(
        Box((0.018, 0.015, 0.062)),
        origin=Origin(xyz=(-0.029, 0.000, 0.235)),
        material=anodized,
        name="battery_pod",
    )
    outer_tube.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(-0.037, 0.000, 0.262), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="charge_cap",
    )
    outer_tube.inertial = Inertial.from_geometry(
        Box((0.085, 0.060, 0.315)),
        mass=0.85,
        origin=Origin(xyz=(-0.010, 0.000, 0.158)),
    )

    inner_tube = model.part("inner_tube")
    inner_tube.visual(
        Cylinder(radius=0.0131, length=0.310),
        origin=Origin(xyz=(0.000, 0.000, 0.055)),
        material=alloy,
        name="inner_stanchion",
    )
    inner_tube.visual(
        Box((0.050, 0.042, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, 0.219)),
        material=dark_gray,
        name="saddle_yoke",
    )
    inner_tube.visual(
        Cylinder(radius=0.0045, length=0.070),
        origin=Origin(xyz=(0.000, -0.018, 0.230), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="left_rail",
    )
    inner_tube.visual(
        Cylinder(radius=0.0045, length=0.070),
        origin=Origin(xyz=(0.000, 0.018, 0.230), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="right_rail",
    )
    inner_tube.inertial = Inertial.from_geometry(
        Box((0.080, 0.090, 0.340)),
        mass=0.75,
        origin=Origin(xyz=(0.000, 0.000, 0.120)),
    )

    remote_body = model.part("remote_body")
    remote_body.visual(
        Sphere(radius=0.0030),
        material=matte_black,
        name="bar_contact_pad",
    )
    remote_body.visual(
        Box((0.008, 0.014, 0.006)),
        origin=Origin(xyz=(0.004, 0.000, -0.006)),
        material=matte_black,
        name="remote_saddle",
    )
    remote_body.visual(
        Box((0.016, 0.012, 0.010)),
        origin=Origin(xyz=(0.012, 0.000, -0.012)),
        material=matte_black,
        name="remote_mount_arm",
    )
    remote_body.visual(
        Box((0.024, 0.018, 0.020)),
        origin=Origin(xyz=(0.022, 0.000, -0.021)),
        material=satin_black,
        name="remote_pod",
    )
    remote_body.visual(
        Box((0.020, 0.011, 0.008)),
        origin=Origin(xyz=(0.031, 0.000, -0.011)),
        material=dark_gray,
        name="hinge_bridge",
    )
    remote_body.visual(
        Box((0.008, 0.004, 0.012)),
        origin=Origin(xyz=(0.041, -0.0075, -0.005)),
        material=dark_gray,
        name="left_hinge_ear",
    )
    remote_body.visual(
        Box((0.008, 0.004, 0.012)),
        origin=Origin(xyz=(0.041, 0.0075, -0.005)),
        material=dark_gray,
        name="right_hinge_ear",
    )
    remote_body.visual(
        Cylinder(radius=0.0030, length=0.010),
        origin=Origin(xyz=(0.022, 0.011, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="status_button",
    )
    remote_body.inertial = Inertial.from_geometry(
        Box((0.055, 0.050, 0.045)),
        mass=0.08,
        origin=Origin(xyz=(0.022, 0.000, -0.016)),
    )

    remote_lever = model.part("remote_lever")
    remote_lever.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="lever_barrel",
    )
    remote_lever.visual(
        Box((0.010, 0.008, 0.010)),
        origin=Origin(xyz=(0.004, 0.000, -0.006)),
        material=dark_gray,
        name="lever_knuckle",
    )
    remote_lever.visual(
        Box((0.010, 0.009, 0.028)),
        origin=Origin(xyz=(0.010, 0.000, -0.021)),
        material=matte_black,
        name="lever_arm",
    )
    remote_lever.visual(
        Box((0.020, 0.016, 0.018)),
        origin=Origin(xyz=(0.016, 0.000, -0.043)),
        material=rubber,
        name="lever_paddle",
    )
    remote_lever.inertial = Inertial.from_geometry(
        Box((0.032, 0.020, 0.058)),
        mass=0.025,
        origin=Origin(xyz=(0.014, 0.000, -0.027)),
    )

    model.articulation(
        "frame_to_outer_tube",
        ArticulationType.FIXED,
        parent=bike_stub,
        child=outer_tube,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
    )
    model.articulation(
        "outer_tube_to_inner_tube",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_tube,
        origin=Origin(xyz=(0.000, 0.000, 0.300)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.20,
            lower=0.0,
            upper=0.170,
        ),
    )
    remote_mount_surface = surface_frame(
        handlebar_visual,
        point_hint=(0.543, -0.176, 0.2965),
    )

    model.articulation(
        "handlebar_to_remote_body",
        ArticulationType.FIXED,
        parent=bike_stub,
        child=remote_body,
        origin=Origin(
            xyz=tuple(
                remote_mount_surface.point[i] + remote_mount_surface.normal[i] * 0.0030005
                for i in range(3)
            ),
        ),
    )
    model.articulation(
        "remote_body_to_lever",
        ArticulationType.REVOLUTE,
        parent=remote_body,
        child=remote_lever,
        origin=Origin(xyz=(0.041, 0.000, -0.004)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=5.0,
            lower=0.0,
            upper=0.78,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bike_stub = object_model.get_part("bike_stub")
    outer_tube = object_model.get_part("outer_tube")
    inner_tube = object_model.get_part("inner_tube")
    remote_body = object_model.get_part("remote_body")
    remote_lever = object_model.get_part("remote_lever")
    drop_joint = object_model.get_articulation("outer_tube_to_inner_tube")
    lever_joint = object_model.get_articulation("remote_body_to_lever")

    drop_limits = drop_joint.motion_limits
    lever_limits = lever_joint.motion_limits

    ctx.check(
        "seatpost slider axis lowers the inner tube on positive travel",
        tuple(drop_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={drop_joint.axis}",
    )
    ctx.check(
        "remote lever pivots on a transverse hinge",
        tuple(lever_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={lever_joint.axis}",
    )

    with ctx.pose({drop_joint: 0.0}):
        ctx.expect_contact(
            outer_tube,
            bike_stub,
            elem_a="outer_sleeve",
            elem_b="upper_post_guide",
            contact_tol=0.0002,
            name="outer tube bears on the frame's upper guide bushing",
        )
        ctx.expect_within(
            outer_tube,
            bike_stub,
            axes="xy",
            inner_elem="outer_sleeve",
            outer_elem="seat_tube",
            margin=0.003,
            name="outer tube stays centered in the frame seat tube",
        )
        ctx.expect_overlap(
            outer_tube,
            bike_stub,
            axes="z",
            elem_a="outer_sleeve",
            elem_b="seat_tube",
            min_overlap=0.100,
            name="outer tube remains deeply inserted in the frame",
        )
        ctx.expect_within(
            inner_tube,
            outer_tube,
            axes="xy",
            inner_elem="inner_stanchion",
            outer_elem="outer_sleeve",
            margin=0.004,
            name="extended stanchion stays coaxial with the outer tube",
        )
        ctx.expect_overlap(
            inner_tube,
            outer_tube,
            axes="z",
            elem_a="inner_stanchion",
            min_overlap=0.090,
            name="extended stanchion retains insertion in the outer tube",
        )
        extended_origin = ctx.part_world_position(inner_tube)

    if drop_limits is not None and drop_limits.upper is not None:
        with ctx.pose({drop_joint: drop_limits.upper}):
            ctx.expect_within(
                inner_tube,
                outer_tube,
                axes="xy",
                inner_elem="inner_stanchion",
                outer_elem="outer_sleeve",
                margin=0.004,
                name="dropped stanchion stays coaxial with the outer tube",
            )
            ctx.expect_overlap(
                inner_tube,
                outer_tube,
                axes="z",
                elem_a="inner_stanchion",
                elem_b="outer_sleeve",
                min_overlap=0.240,
                name="dropped stanchion remains retained inside the outer tube",
            )
            dropped_origin = ctx.part_world_position(inner_tube)

        ctx.check(
            "positive slider travel moves the saddle assembly downward",
            extended_origin is not None
            and dropped_origin is not None
            and dropped_origin[2] < extended_origin[2] - 0.120,
            details=f"extended={extended_origin}, dropped={dropped_origin}",
        )

    def _elem_center(part, elem_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    lever_rest_center = _elem_center(remote_lever, "lever_paddle")
    if lever_limits is not None and lever_limits.upper is not None:
        with ctx.pose({lever_joint: lever_limits.upper}):
            lever_pressed_center = _elem_center(remote_lever, "lever_paddle")
        ctx.check(
            "remote lever paddle sweeps through a clear pressed arc",
            lever_rest_center is not None
            and lever_pressed_center is not None
            and math.dist(lever_rest_center, lever_pressed_center) > 0.010,
            details=f"rest={lever_rest_center}, pressed={lever_pressed_center}",
        )

    ctx.expect_contact(
        remote_body,
        bike_stub,
        elem_a="bar_contact_pad",
        elem_b="handlebar",
        contact_tol=0.0005,
        name="remote body is mounted directly onto the handlebar",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
