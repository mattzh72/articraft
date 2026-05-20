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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = vector
    length = math.sqrt(x * x + y * y + z * z)
    return (x / length, y / length, z / length)


def _scale(vector: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    return (vector[0] * scalar, vector[1] * scalar, vector[2] * scalar)


def _add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    radial_segments: int = 18,
) -> object:
    return mesh_from_geometry(
        wire_from_points(
            points,
            radius=radius,
            radial_segments=radial_segments,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.0,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_studio_floodlight")

    powder_black = model.material("powder_black", rgba=(0.11, 0.11, 0.12, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.36, 0.38, 0.40, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.90, 0.94, 0.97, 0.45))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    hub_z = 0.43
    hub_center = (0.0, 0.0, hub_z)
    leg_joint_offset = 0.03
    leg_sleeve_length = 0.19
    leg_visible_length = 0.66
    leg_extension = 0.15

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.060, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, hub_z)),
        material=powder_black,
        name="hub_crown",
    )
    hub.visual(
        Cylinder(radius=0.045, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, hub_z + 0.055)),
        material=steel_gray,
        name="center_riser",
    )
    hub.visual(
        Box((0.040, 0.050, 0.055)),
        origin=Origin(xyz=(0.020, 0.0, hub_z + 0.055)),
        material=powder_black,
        name="hinge_pedestal",
    )
    hub.visual(
        Box((0.022, 0.012, 0.085)),
        origin=Origin(xyz=(0.058, 0.018, hub_z + 0.070)),
        material=steel_gray,
        name="hinge_plate_left",
    )
    hub.visual(
        Box((0.022, 0.012, 0.085)),
        origin=Origin(xyz=(0.058, -0.018, hub_z + 0.070)),
        material=steel_gray,
        name="hinge_plate_right",
    )
    hub.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.22)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, hub_z + 0.03)),
    )

    leg_axes: list[tuple[float, float, float]] = []
    leg_parts = []
    leg_joint_names = []
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        axis = _normalize((0.78 * math.cos(angle), 0.78 * math.sin(angle), -0.62))
        leg_axes.append(axis)
        joint_origin = _add(hub_center, _scale(axis, leg_joint_offset))
        sleeve_end = _add(joint_origin, _scale(axis, leg_sleeve_length))
        sleeve_start = _add(joint_origin, _scale(axis, -0.025))
        hub.visual(
            _tube_mesh(
                f"hub_leg_sleeve_{index}",
                [sleeve_start, sleeve_end],
                radius=0.020,
            ),
            material=powder_black,
            name=f"leg_sleeve_{index}",
        )

        leg = model.part(f"leg_{index}")
        leg.visual(
            _tube_mesh(
                f"leg_{index}_tube",
                [(0.0, 0.0, 0.0), _scale(axis, leg_visible_length)],
                radius=0.013,
            ),
            material=aluminum,
            name="lower_tube",
        )
        leg.visual(
            _tube_mesh(
                f"leg_{index}_collar",
                [_scale(axis, 0.150), _scale(axis, 0.255)],
                radius=0.0165,
            ),
            material=steel_gray,
            name="lock_collar",
        )
        foot_anchor = _scale(axis, leg_visible_length)
        leg.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(xyz=(foot_anchor[0], foot_anchor[1], foot_anchor[2] - 0.007)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Cylinder(radius=0.018, length=0.72),
            mass=1.0,
            origin=Origin(xyz=(axis[0] * 0.33, axis[1] * 0.33, axis[2] * 0.33)),
        )
        joint_name = f"hub_to_leg_{index}"
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=hub,
            child=leg,
            origin=Origin(xyz=joint_origin),
            axis=axis,
            motion_limits=MotionLimits(
                effort=60.0,
                velocity=0.18,
                lower=0.0,
                upper=leg_extension,
            ),
        )
        leg_parts.append(leg)
        leg_joint_names.append(joint_name)

    upright = model.part("upright")
    upright.visual(
        Box((0.020, 0.024, 0.064)),
        origin=Origin(),
        material=steel_gray,
        name="hinge_tongue",
    )
    upright.visual(
        Box((0.022, 0.024, 0.030)),
        origin=Origin(xyz=(0.010, 0.0, 0.040)),
        material=steel_gray,
        name="hinge_neck",
    )
    upright.visual(
        Box((0.048, 0.050, 0.090)),
        origin=Origin(xyz=(0.022, 0.0, 0.092)),
        material=powder_black,
        name="lower_casting",
    )
    upright.visual(
        Box((0.044, 0.036, 0.700)),
        origin=Origin(xyz=(0.022, 0.0, 0.472)),
        material=aluminum,
        name="mast",
    )
    upright.visual(
        Box((0.082, 0.052, 0.050)),
        origin=Origin(xyz=(0.046, 0.0, 0.844)),
        material=powder_black,
        name="top_cap",
    )
    upright.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, 0.92)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
    )

    model.articulation(
        "hub_to_upright",
        ArticulationType.REVOLUTE,
        parent=hub,
        child=upright,
        origin=Origin(xyz=(0.058, 0.0, hub_z + 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=math.radians(-22.0),
            upper=math.radians(48.0),
        ),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.040, 0.060, 0.070)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=steel_gray,
        name="mount_block",
    )
    yoke.visual(
        Box((0.024, 0.300, 0.270)),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material=steel_gray,
        name="center_frame",
    )
    yoke.visual(
        Box((0.190, 0.344, 0.018)),
        origin=Origin(xyz=(0.095, 0.0, 0.134)),
        material=powder_black,
        name="top_bridge",
    )
    yoke.visual(
        Box((0.190, 0.344, 0.018)),
        origin=Origin(xyz=(0.095, 0.0, -0.134)),
        material=powder_black,
        name="bottom_bridge",
    )
    yoke.visual(
        Box((0.024, 0.012, 0.250)),
        origin=Origin(xyz=(0.180, 0.178, 0.0)),
        material=powder_black,
        name="left_arm",
    )
    yoke.visual(
        Box((0.024, 0.012, 0.250)),
        origin=Origin(xyz=(0.180, -0.178, 0.0)),
        material=powder_black,
        name="right_arm",
    )
    yoke.visual(
        Box((0.024, 0.050, 0.250)),
        origin=Origin(xyz=(0.032, 0.150, 0.0)),
        material=steel_gray,
        name="left_web",
    )
    yoke.visual(
        Box((0.024, 0.050, 0.250)),
        origin=Origin(xyz=(0.032, -0.150, 0.0)),
        material=steel_gray,
        name="right_web",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.24, 0.40, 0.30)),
        mass=1.2,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
    )

    model.articulation(
        "upright_to_yoke",
        ArticulationType.FIXED,
        parent=upright,
        child=yoke,
        origin=Origin(xyz=(0.087, 0.0, 0.847)),
    )

    head = model.part("head")
    head.visual(
        Box((0.012, 0.320, 0.220)),
        origin=Origin(xyz=(-0.086, 0.0, 0.0)),
        material=powder_black,
        name="rear_panel",
    )
    head.visual(
        Box((0.160, 0.010, 0.200)),
        origin=Origin(xyz=(0.0, 0.155, 0.0)),
        material=powder_black,
        name="left_wall",
    )
    head.visual(
        Box((0.160, 0.010, 0.200)),
        origin=Origin(xyz=(0.0, -0.155, 0.0)),
        material=powder_black,
        name="right_wall",
    )
    head.visual(
        Box((0.160, 0.300, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=powder_black,
        name="top_wall",
    )
    head.visual(
        Box((0.160, 0.300, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=powder_black,
        name="bottom_wall",
    )
    head.visual(
        Box((0.016, 0.320, 0.018)),
        origin=Origin(xyz=(0.088, 0.0, 0.101)),
        material=powder_black,
        name="front_bezel_top",
    )
    head.visual(
        Box((0.016, 0.320, 0.018)),
        origin=Origin(xyz=(0.088, 0.0, -0.101)),
        material=powder_black,
        name="front_bezel_bottom",
    )
    head.visual(
        Box((0.016, 0.020, 0.186)),
        origin=Origin(xyz=(0.088, 0.150, 0.0)),
        material=powder_black,
        name="front_bezel_left",
    )
    head.visual(
        Box((0.016, 0.020, 0.186)),
        origin=Origin(xyz=(0.088, -0.150, 0.0)),
        material=powder_black,
        name="front_bezel_right",
    )
    head.visual(
        Box((0.006, 0.302, 0.184)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=lens_glass,
        name="diffuser",
    )
    head.visual(
        Box((0.028, 0.320, 0.014)),
        origin=Origin(xyz=(0.100, 0.0, 0.117)),
        material=powder_black,
        name="top_visor",
    )
    for fin_index, fin_y in enumerate((-0.100, -0.050, 0.0, 0.050, 0.100)):
        head.visual(
            Box((0.048, 0.016, 0.176)),
            origin=Origin(xyz=(-0.116, fin_y, 0.0)),
            material=steel_gray,
            name=f"cooling_fin_{fin_index}",
        )
    head.visual(
        Box((0.040, 0.120, 0.072)),
        origin=Origin(xyz=(-0.112, 0.0, -0.042)),
        material=steel_gray,
        name="driver_box",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.166, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.166, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="right_trunnion",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.22, 0.34, 0.24)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=1.4,
            lower=math.radians(-78.0),
            upper=math.radians(52.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    hub = object_model.get_part("hub")
    upright = object_model.get_part("upright")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head")
    legs = [object_model.get_part(f"leg_{index}") for index in range(1, 4)]

    leg_joints = [object_model.get_articulation(f"hub_to_leg_{index}") for index in range(1, 4)]
    hub_to_upright = object_model.get_articulation("hub_to_upright")
    yoke_to_head = object_model.get_articulation("yoke_to_head")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
    for leg in legs:
        ctx.allow_overlap(
            hub,
            leg,
            reason="Telescoping leg tube intentionally rides inside the hub sleeve.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    for leg in legs:
        ctx.expect_contact(leg, hub)
    ctx.expect_contact(upright, hub)
    ctx.expect_contact(yoke, upright)
    ctx.expect_contact(head, yoke)

    hub_pos = ctx.part_world_position(hub)
    assert hub_pos is not None

    for leg, leg_joint in zip(legs, leg_joints):
        leg_rest = ctx.part_world_position(leg)
        assert leg_rest is not None
        rest_xy = math.hypot(leg_rest[0] - hub_pos[0], leg_rest[1] - hub_pos[1])
        with ctx.pose({leg_joint: 0.12}):
            leg_extended = ctx.part_world_position(leg)
            assert leg_extended is not None
            ext_xy = math.hypot(leg_extended[0] - hub_pos[0], leg_extended[1] - hub_pos[1])
            assert ext_xy > rest_xy + 0.08
            assert leg_extended[2] < leg_rest[2] - 0.06

    yoke_rest = ctx.part_world_position(yoke)
    assert yoke_rest is not None
    with ctx.pose({hub_to_upright: math.radians(35.0)}):
        yoke_tilted = ctx.part_world_position(yoke)
        assert yoke_tilted is not None
        assert yoke_tilted[0] > yoke_rest[0] + 0.40
        assert yoke_tilted[2] < yoke_rest[2] - 0.10
        ctx.expect_contact(upright, hub)

    diffuser_rest = ctx.part_element_world_aabb(head, elem="diffuser")
    assert diffuser_rest is not None
    rest_center_z = (diffuser_rest[0][2] + diffuser_rest[1][2]) * 0.5
    rest_center_x = (diffuser_rest[0][0] + diffuser_rest[1][0]) * 0.5
    with ctx.pose({yoke_to_head: math.radians(40.0)}):
        diffuser_tilted = ctx.part_element_world_aabb(head, elem="diffuser")
        assert diffuser_tilted is not None
        tilt_center_z = (diffuser_tilted[0][2] + diffuser_tilted[1][2]) * 0.5
        tilt_center_x = (diffuser_tilted[0][0] + diffuser_tilted[1][0]) * 0.5
        assert tilt_center_z < rest_center_z - 0.040
        assert tilt_center_x < rest_center_x - 0.015
        ctx.expect_contact(head, yoke)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
