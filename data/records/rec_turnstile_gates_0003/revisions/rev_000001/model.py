from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_turnstile_gate", assets=ASSETS)

    matte_anthracite = model.material(
        "matte_anthracite",
        rgba=(0.17, 0.18, 0.20, 1.0),
    )
    matte_graphite = model.material(
        "matte_graphite",
        rgba=(0.23, 0.24, 0.26, 1.0),
    )
    satin_graphite = model.material(
        "satin_graphite",
        rgba=(0.35, 0.37, 0.40, 1.0),
    )
    satin_stainless = model.material(
        "satin_stainless",
        rgba=(0.74, 0.76, 0.79, 1.0),
    )
    elastomer_black = model.material(
        "elastomer_black",
        rgba=(0.08, 0.08, 0.09, 1.0),
    )

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def _add_radial_cylinder(
        part,
        *,
        name: str,
        radius: float,
        length: float,
        center_radius: float,
        angle: float,
        z: float,
        material,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=(
                    math.cos(angle) * center_radius,
                    math.sin(angle) * center_radius,
                    z,
                ),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=material,
            name=name,
        )

    def _rounded_prism_mesh(
        name: str,
        *,
        size: tuple[float, float, float],
        radius: float,
        axis: str = "z",
    ):
        sx, sy, sz = size
        if axis == "z":
            geometry = ExtrudeGeometry(
                rounded_rect_profile(sx, sy, radius, corner_segments=10),
                sz,
                center=True,
            )
        elif axis == "x":
            geometry = ExtrudeGeometry(
                rounded_rect_profile(sz, sy, radius, corner_segments=10),
                sx,
                center=True,
            )
            geometry.rotate_y(math.pi / 2.0)
        else:
            raise ValueError(f"Unsupported prism axis: {axis}")
        return _save_mesh(name, geometry)

    plinth_mesh = _save_mesh(
        "turnstile_plinth.obj",
        ExtrudeGeometry(
            rounded_rect_profile(1.46, 0.42, 0.045, corner_segments=10),
            0.108,
            center=True,
        ),
    )
    top_cap_mesh = _save_mesh(
        "turnstile_top_cap.obj",
        ExtrudeGeometry(
            rounded_rect_profile(1.34, 0.34, 0.032, corner_segments=10),
            0.012,
            center=True,
        ),
    )
    mast_body_mesh = _rounded_prism_mesh(
        "turnstile_mast_body.obj",
        size=(0.18, 0.12, 0.84),
        radius=0.026,
    )
    mast_fascia_mesh = _rounded_prism_mesh(
        "turnstile_mast_fascia.obj",
        size=(0.018, 0.108, 0.62),
        radius=0.006,
    )
    mast_crown_mesh = _rounded_prism_mesh(
        "turnstile_mast_crown.obj",
        size=(0.20, 0.16, 0.03),
        radius=0.018,
    )
    bridge_body_mesh = _rounded_prism_mesh(
        "turnstile_bridge_body.obj",
        size=(0.63, 0.14, 0.08),
        radius=0.022,
        axis="x",
    )
    bridge_underside_mesh = _rounded_prism_mesh(
        "turnstile_bridge_underside.obj",
        size=(0.50, 0.11, 0.012),
        radius=0.003,
        axis="x",
    )
    bridge_nose_mesh = _rounded_prism_mesh(
        "turnstile_bridge_nose.obj",
        size=(0.035, 0.14, 0.095),
        radius=0.016,
        axis="x",
    )

    frame = model.part("frame")
    frame.visual(
        plinth_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=matte_anthracite,
        name="base_plinth",
    )
    frame.visual(
        top_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        material=matte_graphite,
        name="top_cap",
    )
    frame.visual(
        mast_body_mesh,
        origin=Origin(xyz=(-0.63, 0.0, 0.54)),
        material=matte_anthracite,
        name="mast_body",
    )
    frame.visual(
        mast_fascia_mesh,
        origin=Origin(xyz=(-0.536, 0.0, 0.54)),
        material=satin_graphite,
        name="mast_fascia",
    )
    frame.visual(
        mast_crown_mesh,
        origin=Origin(xyz=(-0.63, 0.0, 0.975)),
        material=matte_graphite,
        name="mast_crown",
    )
    frame.visual(
        bridge_body_mesh,
        origin=Origin(xyz=(-0.225, 0.0, 1.03)),
        material=matte_anthracite,
        name="bridge_body",
    )
    frame.visual(
        bridge_underside_mesh,
        origin=Origin(xyz=(-0.17, 0.0, 0.996)),
        material=satin_graphite,
        name="bridge_underside",
    )
    frame.visual(
        bridge_nose_mesh,
        origin=Origin(xyz=(0.0725, 0.0, 1.0225)),
        material=satin_graphite,
        name="bridge_nose_plate",
    )
    frame.visual(
        Box((0.006, 0.072, 0.22)),
        origin=Origin(xyz=(-0.538, 0.0, 0.63)),
        material=satin_stainless,
        name="reader_strip",
    )
    frame.visual(
        Box((0.004, 0.088, 0.25)),
        origin=Origin(xyz=(-0.539, 0.0, 0.33)),
        material=satin_graphite,
        name="service_hatch",
    )
    frame.visual(
        Cylinder(radius=0.054, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=satin_graphite,
        name="central_post",
    )
    frame.visual(
        Cylinder(radius=0.072, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=satin_graphite,
        name="lower_bearing_body",
    )
    frame.visual(
        Cylinder(radius=0.094, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=satin_stainless,
        name="lower_bearing_pad",
    )
    frame.visual(
        Cylinder(radius=0.072, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.975)),
        material=satin_graphite,
        name="upper_bearing_body",
    )
    frame.visual(
        Cylinder(radius=0.094, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.955)),
        material=satin_stainless,
        name="upper_bearing_pad",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.979)),
        material=satin_stainless,
        name="upper_retainer_cap",
    )
    for x in (-0.16, 0.16):
        frame.visual(
            Cylinder(radius=0.012, length=0.01),
            origin=Origin(xyz=(x, 0.16, 0.125)),
            material=satin_stainless,
            name=f"deck_fastener_front_{'left' if x < 0.0 else 'right'}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.01),
            origin=Origin(xyz=(x, -0.16, 0.125)),
            material=satin_stainless,
            name=f"deck_fastener_rear_{'left' if x < 0.0 else 'right'}",
        )
    frame.inertial = Inertial.from_geometry(
        Box((1.46, 0.42, 1.11)),
        mass=185.0,
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.088, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
        material=satin_stainless,
        name="lower_thrust_face",
    )
    rotor.visual(
        Cylinder(radius=0.066, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=satin_graphite,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.088, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=satin_stainless,
        name="upper_hub_cap",
    )
    rotor.visual(
        Cylinder(radius=0.034, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=matte_graphite,
        name="inner_hub_core",
    )
    arm_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(arm_angles):
        _add_radial_cylinder(
            rotor,
            name=f"arm_{index}_socket",
            radius=0.028,
            length=0.12,
            center_radius=0.11,
            angle=angle,
            z=0.0,
            material=satin_graphite,
        )
        _add_radial_cylinder(
            rotor,
            name=f"arm_{index}_tube",
            radius=0.019,
            length=0.34,
            center_radius=0.31,
            angle=angle,
            z=0.0,
            material=satin_stainless,
        )
        _add_radial_cylinder(
            rotor,
            name=f"arm_{index}_end_sleeve",
            radius=0.022,
            length=0.05,
            center_radius=0.505,
            angle=angle,
            z=0.0,
            material=elastomer_black,
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.54, length=0.10),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "hub_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=3.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    hub_rotation = object_model.get_articulation("hub_rotation")

    lower_bearing_pad = frame.get_visual("lower_bearing_pad")
    upper_bearing_pad = frame.get_visual("upper_bearing_pad")
    bridge_body = frame.get_visual("bridge_body")
    mast_body = frame.get_visual("mast_body")
    lower_thrust_face = rotor.get_visual("lower_thrust_face")
    upper_hub_cap = rotor.get_visual("upper_hub_cap")
    arm_0_tube = rotor.get_visual("arm_0_tube")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_isolated_parts(max_pose_samples=12, name="supported_in_sampled_rotations")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    limits = hub_rotation.motion_limits
    axis_ok = hub_rotation.axis == (0.0, 0.0, 1.0)
    continuous_ok = (
        limits is not None
        and limits.lower is None
        and limits.upper is None
        and limits.velocity > 0.0
    )
    ctx.check("hub_rotation_axis_vertical", axis_ok, f"axis={hub_rotation.axis}")
    ctx.check(
        "hub_rotation_is_continuous",
        continuous_ok,
        (
            "Continuous turnstile rotor should have unbounded limits; "
            f"limits={limits}"
        ),
    )

    ctx.expect_contact(
        rotor,
        frame,
        elem_a=lower_thrust_face,
        elem_b=lower_bearing_pad,
        name="lower_bearing_support_contact",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a=upper_hub_cap,
        elem_b=upper_bearing_pad,
        name="upper_bearing_support_contact",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem=bridge_body,
        negative_elem=upper_hub_cap,
        min_gap=0.035,
        max_gap=0.065,
        name="bridge_clearance_above_hub",
    )

    arm_rest_aabb = ctx.part_element_world_aabb(rotor, elem=arm_0_tube)
    if arm_rest_aabb is None:
        ctx.fail("arm_0_rest_aabb", "arm_0_tube AABB unavailable in rest pose")
    else:
        with ctx.pose({hub_rotation: 2.0 * math.pi / 3.0}):
            ctx.expect_contact(
                rotor,
                frame,
                elem_a=lower_thrust_face,
                elem_b=lower_bearing_pad,
                name="lower_bearing_contact_rotated",
            )
            ctx.expect_contact(
                rotor,
                frame,
                elem_a=upper_hub_cap,
                elem_b=upper_bearing_pad,
                name="upper_bearing_contact_rotated",
            )
            arm_rot_aabb = ctx.part_element_world_aabb(rotor, elem=arm_0_tube)
            if arm_rot_aabb is None:
                ctx.fail("arm_0_rotated_aabb", "arm_0_tube AABB unavailable after rotation")
            else:
                rest_center = (
                    (arm_rest_aabb[0][0] + arm_rest_aabb[1][0]) * 0.5,
                    (arm_rest_aabb[0][1] + arm_rest_aabb[1][1]) * 0.5,
                )
                rot_center = (
                    (arm_rot_aabb[0][0] + arm_rot_aabb[1][0]) * 0.5,
                    (arm_rot_aabb[0][1] + arm_rot_aabb[1][1]) * 0.5,
                )
                moved = (
                    abs(rot_center[0] - rest_center[0]) > 0.18
                    or abs(rot_center[1] - rest_center[1]) > 0.18
                )
                ctx.check(
                    "arm_moves_with_rotor",
                    moved,
                    (
                        "Turnstile arm did not move enough under hub rotation; "
                        f"rest_center={rest_center}, rotated_center={rot_center}"
                    ),
                )

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is None:
        ctx.fail("frame_aabb_available", "Frame AABB unavailable")
    else:
        frame_size = tuple(
            frame_aabb[1][axis] - frame_aabb[0][axis] for axis in range(3)
        )
        ctx.check(
            "frame_proportions_realistic",
            1.40 <= frame_size[0] <= 1.55
            and 0.40 <= frame_size[1] <= 0.48
            and 1.05 <= frame_size[2] <= 1.10,
            f"frame_size={frame_size}",
        )

    with ctx.pose({hub_rotation: math.pi / 6.0}):
        rotor_aabb = ctx.part_world_aabb(rotor)
        if rotor_aabb is None:
            ctx.fail("rotor_aabb_available", "Rotor AABB unavailable")
        else:
            rotor_size = tuple(
                rotor_aabb[1][axis] - rotor_aabb[0][axis] for axis in range(3)
            )
            ctx.check(
                "arm_span_realistic",
                0.86 <= max(rotor_size[0], rotor_size[1]) <= 0.96,
                f"rotor_size={rotor_size}",
            )
            ctx.expect_gap(
                rotor,
                frame,
                axis="x",
                positive_elem=arm_0_tube,
                negative_elem=mast_body,
                min_gap=0.10,
                name="mast_clear_of_rotor_sweep_pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
