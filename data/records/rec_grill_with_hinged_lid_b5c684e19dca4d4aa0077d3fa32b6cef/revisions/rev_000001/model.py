from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _band_mesh(*, inner_radius: float, outer_radius: float, z0: float, z1: float, segments: int = 56):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _wheel_part(part, *, rubber, steel) -> None:
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=spin_origin,
        material=rubber,
        name="wheel_tire",
    )
    part.visual(
        Cylinder(radius=0.031, length=0.024),
        origin=spin_origin,
        material=steel,
        name="wheel_hub",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=spin_origin,
        material=steel,
        name="wheel_axle_bore",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kamado_grill_cart")

    ceramic_green = model.material("ceramic_green", rgba=(0.13, 0.20, 0.14, 1.0))
    cart_black = model.material("cart_black", rgba=(0.10, 0.10, 0.11, 1.0))
    band_metal = model.material("band_metal", rgba=(0.31, 0.32, 0.34, 1.0))
    vent_metal = model.material("vent_metal", rgba=(0.42, 0.43, 0.44, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    handle_wood = model.material("handle_wood", rgba=(0.46, 0.30, 0.16, 1.0))
    thermometer_face = model.material("thermometer_face", rgba=(0.92, 0.93, 0.94, 1.0))
    oval_y_scale = 0.86

    cart_frame = model.part("cart_frame")
    cart_frame.inertial = Inertial.from_geometry(
        Box((0.74, 0.72, 0.62)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )

    # Lower rolling frame.
    cart_frame.visual(
        Box((0.56, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.30, 0.12)),
        material=cart_black,
    )
    cart_frame.visual(
        Box((0.56, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, -0.30, 0.12)),
        material=cart_black,
    )
    cart_frame.visual(
        Box((0.03, 0.60, 0.03)),
        origin=Origin(xyz=(0.265, 0.0, 0.12)),
        material=cart_black,
    )
    cart_frame.visual(
        Box((0.03, 0.60, 0.03)),
        origin=Origin(xyz=(-0.265, 0.0, 0.12)),
        material=cart_black,
    )
    cart_frame.visual(
        Box((0.56, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=cart_black,
    )
    cart_frame.visual(
        Box((0.03, 0.60, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=cart_black,
    )
    cart_frame.visual(
        _save_mesh(
            "cart_support_tray",
            CylinderGeometry(radius=0.13, height=0.02, radial_segments=48).scale(1.0, oval_y_scale, 1.0),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=cart_black,
        name="cart_support_tray",
    )

    # Corner uprights.
    for x in (-0.265, 0.265):
        for y in (-0.30, 0.30):
            cart_frame.visual(
                Box((0.03, 0.03, 0.46)),
                origin=Origin(xyz=(x, y, 0.35)),
                material=cart_black,
            )

    # Wheel axle blocks.
    for x in (-0.265, 0.265):
        for y_sign in (-1.0, 1.0):
            cart_frame.visual(
                Box((0.055, 0.050, 0.100)),
                origin=Origin(xyz=(x, y_sign * 0.327, 0.085)),
                material=cart_black,
            )

    # Upper cradle rails and hoop-like side supports.
    left_rail = tube_from_spline_points(
        [
            (-0.245, 0.30, 0.33),
            (-0.19, 0.31, 0.46),
            (-0.08, 0.315, 0.56),
            (0.08, 0.315, 0.56),
            (0.19, 0.31, 0.46),
            (0.245, 0.30, 0.33),
        ],
        radius=0.012,
        samples_per_segment=16,
        radial_segments=18,
    )
    right_rail = tube_from_spline_points(
        [
            (-0.245, -0.30, 0.33),
            (-0.19, -0.31, 0.46),
            (-0.08, -0.315, 0.56),
            (0.08, -0.315, 0.56),
            (0.19, -0.31, 0.46),
            (0.245, -0.30, 0.33),
        ],
        radius=0.012,
        samples_per_segment=16,
        radial_segments=18,
    )
    cart_frame.visual(_save_mesh("cart_left_cradle_rail", left_rail), material=cart_black)
    cart_frame.visual(_save_mesh("cart_right_cradle_rail", right_rail), material=cart_black)

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.29, length=0.48),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, -0.23)),
    )
    body_shell = LatheGeometry.from_shell_profiles(
        [
            (0.000, -0.460),
            (0.105, -0.460),
            (0.165, -0.430),
            (0.225, -0.335),
            (0.272, -0.205),
            (0.286, -0.090),
            (0.268, -0.030),
            (0.242, 0.000),
        ],
        [
            (0.000, -0.385),
            (0.075, -0.385),
            (0.135, -0.360),
            (0.193, -0.270),
            (0.228, -0.155),
            (0.227, -0.055),
            (0.218, 0.000),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).scale(1.0, oval_y_scale, 1.0)
    body.visual(_save_mesh("kamado_body_shell", body_shell), material=ceramic_green, name="body_shell")
    body.visual(
        _save_mesh(
            "kamado_body_band",
            _band_mesh(inner_radius=0.278, outer_radius=0.292, z0=-0.115, z1=-0.040).scale(1.0, oval_y_scale, 1.0),
        ),
        material=band_metal,
        name="body_band",
    )
    body.visual(
        Box((0.025, 0.11, 0.17)),
        origin=Origin(xyz=(-0.305, 0.095, 0.045)),
        material=band_metal,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.025, 0.11, 0.17)),
        origin=Origin(xyz=(-0.305, -0.095, 0.045)),
        material=band_metal,
        name="right_hinge_bracket",
    )
    body.visual(
        Box((0.055, 0.23, 0.030)),
        origin=Origin(xyz=(-0.282, 0.0, -0.032)),
        material=band_metal,
    )
    body.visual(
        Box((0.028, 0.140, 0.065)),
        origin=Origin(xyz=(0.255, 0.0, -0.235)),
        material=vent_metal,
        name="bottom_draft_door",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.155),
        origin=Origin(xyz=(0.270, 0.0, -0.235), rpy=(0.0, pi / 2.0, 0.0)),
        material=band_metal,
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.30, length=0.42),
        mass=24.0,
        origin=Origin(xyz=(0.255, 0.0, 0.12)),
    )
    lid_shell = LatheGeometry.from_shell_profiles(
        [
            (0.272, 0.000),
            (0.287, 0.070),
            (0.284, 0.175),
            (0.244, 0.285),
            (0.148, 0.360),
            (0.058, 0.392),
            (0.000, 0.398),
        ],
        [
            (0.250, 0.000),
            (0.262, 0.065),
            (0.258, 0.165),
            (0.221, 0.272),
            (0.130, 0.340),
            (0.050, 0.368),
            (0.000, 0.374),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).scale(1.0, oval_y_scale, 1.0)
    lid.visual(
        _save_mesh("kamado_lid_shell", lid_shell),
        origin=Origin(xyz=(0.248, 0.0, -0.040)),
        material=ceramic_green,
        name="lid_shell",
    )
    lid.visual(
        _save_mesh(
            "kamado_lid_band",
            _band_mesh(inner_radius=0.282, outer_radius=0.295, z0=-0.010, z1=0.075).scale(1.0, oval_y_scale, 1.0),
        ),
        origin=Origin(xyz=(0.248, 0.0, -0.040)),
        material=band_metal,
        name="lid_band",
    )
    lid.visual(
        Cylinder(radius=0.048, length=0.034),
        origin=Origin(xyz=(0.248, 0.0, 0.328)),
        material=vent_metal,
        name="vent_collar",
    )
    lid.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(0.248, 0.0, 0.352)),
        material=band_metal,
    )
    lid.visual(
        Box((0.020, 0.028, 0.095)),
        origin=Origin(xyz=(0.470, 0.102, 0.048)),
        material=band_metal,
    )
    lid.visual(
        Box((0.020, 0.028, 0.095)),
        origin=Origin(xyz=(0.470, -0.102, 0.048)),
        material=band_metal,
    )
    lid.visual(
        Cylinder(radius=0.022, length=0.235),
        origin=Origin(xyz=(0.500, 0.0, 0.048), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_wood,
        name="lid_handle",
    )
    lid.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.505, 0.0, 0.140), rpy=(0.0, pi / 2.0, 0.0)),
        material=thermometer_face,
        name="thermometer_dial",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.514, 0.0, 0.140), rpy=(0.0, pi / 2.0, 0.0)),
        material=band_metal,
    )

    vent_cap = model.part("vent_cap")
    vent_cap.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 0.03)),
        mass=0.6,
        origin=Origin(xyz=(0.042, 0.0, 0.008)),
    )
    vent_cap.visual(
        Cylinder(radius=0.058, length=0.008),
        origin=Origin(xyz=(0.042, 0.0, 0.006)),
        material=vent_metal,
        name="vent_cap_disc",
    )
    vent_cap.visual(
        Box((0.030, 0.080, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.006)),
        material=vent_metal,
        name="vent_cap_arm",
    )
    vent_cap.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(xyz=(0.078, 0.0, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=vent_metal,
    )

    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.055, length=0.032),
            mass=0.85,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        _wheel_part(wheel, rubber=wheel_rubber, steel=wheel_steel)

    model.articulation(
        "cart_to_body",
        ArticulationType.FIXED,
        parent=cart_frame,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.24968, 0.0, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=radians(82.0),
        ),
    )
    model.articulation(
        "lid_to_vent_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.210, 0.0, 0.359)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=radians(55.0),
        ),
    )

    wheel_positions = {
        "front_left_wheel": (0.265, 0.367, 0.055),
        "front_right_wheel": (0.265, -0.367, 0.055),
        "rear_left_wheel": (-0.265, 0.367, 0.055),
        "rear_right_wheel": (-0.265, -0.367, 0.055),
    }
    for wheel_name, xyz in wheel_positions.items():
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=cart_frame,
            child=wheel_name,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=25.0),
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

    cart = object_model.get_part("cart_frame")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    vent_cap = object_model.get_part("vent_cap")
    lid_hinge = object_model.get_articulation("body_to_lid")
    vent_hinge = object_model.get_articulation("lid_to_vent_cap")

    ctx.expect_contact(
        body,
        cart,
        elem_a="body_shell",
        elem_b="cart_support_tray",
        name="ceramic body sits on the cart tray",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.006,
            max_penetration=0.0,
            name="closed lid seats cleanly on the bowl",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.40,
            name="lid covers the ceramic bowl opening",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward on the rear hinge band",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_vent_aabb = ctx.part_element_world_aabb(vent_cap, elem="vent_cap_disc")
    with ctx.pose({vent_hinge: vent_hinge.motion_limits.upper}):
        open_vent_aabb = ctx.part_element_world_aabb(vent_cap, elem="vent_cap_disc")
    ctx.check(
        "vent cap pivots open above the crown vent",
        closed_vent_aabb is not None
        and open_vent_aabb is not None
        and open_vent_aabb[1][2] > closed_vent_aabb[1][2] + 0.015,
        details=f"closed={closed_vent_aabb}, open={open_vent_aabb}",
    )

    ctx.check(
        "lid hinge axis points across the grill width",
        tuple(lid_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "vent hinge axis points across the vent",
        tuple(vent_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={vent_hinge.axis}",
    )

    for joint_name in (
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    ):
        wheel_joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} spins on a horizontal axle",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
