from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _deck_loop(
    *,
    x_station: float,
    width: float,
    thickness: float,
    concave: float,
    bottom_crown: float,
    rail_lift: float,
    z_shift: float,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    inner = half_width * 0.64
    outer = half_width * 0.88
    top = thickness * 0.5
    bottom = -thickness * 0.5
    return [
        (x_station, 0.0, z_shift + top - concave),
        (x_station, inner * 0.45, z_shift + top - concave * 0.72),
        (x_station, inner, z_shift + top - concave * 0.18),
        (x_station, outer, z_shift + top + rail_lift * 0.55),
        (x_station, half_width, z_shift + top - 0.0009),
        (x_station, half_width, z_shift + 0.0),
        (x_station, outer, z_shift + bottom + bottom_crown * 0.22),
        (x_station, inner, z_shift + bottom + bottom_crown * 0.72),
        (x_station, 0.0, z_shift + bottom + bottom_crown),
        (x_station, -inner, z_shift + bottom + bottom_crown * 0.72),
        (x_station, -outer, z_shift + bottom + bottom_crown * 0.22),
        (x_station, -half_width, z_shift + 0.0),
        (x_station, -half_width, z_shift + top - 0.0009),
        (x_station, -outer, z_shift + top + rail_lift * 0.55),
        (x_station, -inner, z_shift + top - concave * 0.18),
        (x_station, -inner * 0.45, z_shift + top - concave * 0.72),
    ]


def _deck_mesh(name: str, stations: list[tuple[float, float, float, float, float, float, float]]):
    sections = [
        _deck_loop(
            x_station=x_station,
            width=width,
            thickness=thickness,
            concave=concave,
            bottom_crown=bottom_crown,
            rail_lift=rail_lift,
            z_shift=z_shift,
        )
        for x_station, width, thickness, concave, bottom_crown, rail_lift, z_shift in stations
    ]
    return _mesh(name, section_loft(sections))


def _annular_lathe_mesh(name: str, outer_profile, inner_profile, *, segments: int = 56):
    return _mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )


def _wheel_meshes():
    wheel_outer = _annular_lathe_mesh(
        "premium_skateboard_wheel_outer.obj",
        [
            (0.0210, -0.0155),
            (0.0248, -0.0120),
            (0.0260, -0.0030),
            (0.0262, 0.0000),
            (0.0260, 0.0030),
            (0.0248, 0.0120),
            (0.0210, 0.0155),
        ],
        [(0.0054, -0.0155), (0.0054, 0.0155)],
        segments=60,
    )
    wheel_core = _annular_lathe_mesh(
        "premium_skateboard_wheel_core.obj",
        [
            (0.0135, -0.0105),
            (0.0158, -0.0070),
            (0.0165, 0.0000),
            (0.0158, 0.0070),
            (0.0135, 0.0105),
        ],
        [(0.0054, -0.0105), (0.0054, 0.0105)],
        segments=48,
    )
    return wheel_outer, wheel_core


def _truck_interface_meshes():
    sleeve = _annular_lathe_mesh(
        "premium_skateboard_hanger_sleeve.obj",
        [(0.0088, -0.0011), (0.0094, 0.0000), (0.0088, 0.0011)],
        [(0.0037, -0.0011), (0.0037, 0.0011)],
        segments=52,
    )
    return sleeve, None


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_skateboard", assets=ASSETS)

    deck_wood = model.material("deck_wood", rgba=(0.60, 0.46, 0.31, 1.0))
    grip_matte = model.material("grip_matte", rgba=(0.08, 0.09, 0.10, 1.0))
    inlay_satin = model.material("inlay_satin", rgba=(0.19, 0.21, 0.23, 1.0))
    cast_aluminum = model.material("cast_aluminum", rgba=(0.71, 0.73, 0.75, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.52, 0.55, 0.59, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    bushing_smoke = model.material("bushing_smoke", rgba=(0.34, 0.35, 0.37, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.87, 0.84, 0.79, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.30, 0.31, 0.33, 1.0))
    riser_rubber = model.material("riser_rubber", rgba=(0.09, 0.10, 0.11, 1.0))

    deck_body_mesh = _deck_mesh(
        "premium_skateboard_deck.obj",
        [
            (-0.410, 0.086, 0.0108, 0.0006, 0.0002, 0.0001, 0.034),
            (-0.352, 0.138, 0.0112, 0.0010, 0.0003, 0.0002, 0.020),
            (-0.272, 0.190, 0.0118, 0.0018, 0.0005, 0.0005, 0.006),
            (-0.190, 0.214, 0.0120, 0.0024, 0.0007, 0.0008, 0.000),
            (-0.080, 0.216, 0.0120, 0.0026, 0.0008, 0.0010, 0.000),
            (0.080, 0.216, 0.0120, 0.0026, 0.0008, 0.0010, 0.000),
            (0.190, 0.214, 0.0120, 0.0024, 0.0007, 0.0008, 0.000),
            (0.276, 0.198, 0.0118, 0.0019, 0.0005, 0.0006, 0.008),
            (0.356, 0.146, 0.0112, 0.0010, 0.0003, 0.0002, 0.024),
            (0.410, 0.100, 0.0108, 0.0006, 0.0002, 0.0001, 0.041),
        ],
    )
    grip_mesh = _deck_mesh(
        "premium_skateboard_grip.obj",
        [
            (-0.362, 0.122, 0.0014, 0.0008, 0.0000, 0.0002, 0.0260),
            (-0.270, 0.180, 0.0014, 0.0015, 0.0000, 0.0004, 0.0122),
            (-0.190, 0.198, 0.0014, 0.0018, 0.0000, 0.0004, 0.0066),
            (-0.080, 0.200, 0.0014, 0.0020, 0.0000, 0.0004, 0.0066),
            (0.080, 0.200, 0.0014, 0.0020, 0.0000, 0.0004, 0.0066),
            (0.190, 0.198, 0.0014, 0.0018, 0.0000, 0.0004, 0.0066),
            (0.272, 0.182, 0.0014, 0.0015, 0.0000, 0.0004, 0.0142),
            (0.360, 0.128, 0.0014, 0.0008, 0.0000, 0.0002, 0.0320),
        ],
    )
    inlay_mesh = _deck_mesh(
        "premium_skateboard_inlay.obj",
        [
            (-0.246, 0.140, 0.0010, 0.0002, 0.0000, 0.0000, -0.0058),
            (-0.120, 0.152, 0.0010, 0.0002, 0.0000, 0.0000, -0.0058),
            (0.000, 0.160, 0.0010, 0.0002, 0.0000, 0.0000, -0.0058),
            (0.120, 0.152, 0.0010, 0.0002, 0.0000, 0.0000, -0.0058),
            (0.246, 0.140, 0.0010, 0.0002, 0.0000, 0.0000, -0.0058),
        ],
    )
    wheel_outer_mesh, wheel_core_mesh = _wheel_meshes()
    pivot_sleeve_mesh, _ = _truck_interface_meshes()

    deck = model.part("deck")
    deck.visual(deck_body_mesh, material=deck_wood, name="deck_body")
    deck.visual(grip_mesh, material=grip_matte, name="grip_tape")
    deck.visual(inlay_mesh, material=inlay_satin, name="bottom_inlay")
    for mount_x in (-0.182, 0.182):
        for dx in (-0.016, 0.016):
            for mount_y in (-0.016, 0.016):
                deck.visual(
                    Cylinder(radius=0.0032, length=0.0016),
                    origin=Origin(xyz=(mount_x + dx, mount_y, 0.0055)),
                    material=hardware_dark,
                    name=f"bolt_head_{'rear' if mount_x < 0.0 else 'front'}_{dx:+.3f}_{mount_y:+.3f}",
                )
    deck.inertial = Inertial.from_geometry(
        Box((0.820, 0.216, 0.060)),
        mass=1.45,
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
    )

    def add_truck(prefix: str, *, x_pos: float, pitch: float, centerward_sign: float) -> None:
        axis_x = sin(pitch)
        axis_z = cos(pitch)
        kingpin_radius = 0.0028
        baseplate_mount_z = -0.0063
        joint_z = -0.0105
        axle_z = -0.0425
        wheel_center_y = 0.0890
        wheel_half_width = 0.0155
        wheel_inner_face = wheel_center_y - wheel_half_width
        wheel_outer_face = wheel_center_y + wheel_half_width

        baseplate = model.part(f"{prefix}_baseplate")
        baseplate.visual(
            Box((0.078, 0.056, 0.0026)),
            origin=Origin(xyz=(0.000, 0.000, -0.0013)),
            material=riser_rubber,
            name="riser_pad",
        )
        baseplate.visual(
            Box((0.060, 0.048, 0.0046)),
            origin=Origin(xyz=(0.000, 0.000, -0.0049)),
            material=cast_aluminum,
            name="base_plate",
        )
        baseplate.visual(
            Box((0.032, 0.024, 0.0100)),
            origin=Origin(xyz=(0.000, 0.000, -0.0110)),
            material=cast_aluminum,
            name="base_stem",
        )
        baseplate.visual(
            Box((0.020, 0.018, 0.0100)),
            origin=Origin(xyz=(0.012 * centerward_sign, 0.000, -0.0130)),
            material=cast_aluminum,
            name="pivot_housing",
        )
        baseplate.visual(
            Cylinder(radius=kingpin_radius, length=0.0160),
            origin=Origin(
                xyz=(axis_x * 0.0012, 0.000, joint_z + axis_z * 0.0012),
                rpy=(0.000, pitch, 0.000),
            ),
            material=satin_steel,
            name="kingpin_core",
        )
        baseplate.visual(
            Cylinder(radius=0.0090, length=0.0044),
            origin=Origin(
                xyz=(axis_x * 0.0033, 0.000, joint_z + axis_z * 0.0033),
                rpy=(0.000, pitch, 0.000),
            ),
            material=bushing_smoke,
            name="upper_bushing",
        )
        baseplate.visual(
            Cylinder(radius=0.0084, length=0.0044),
            origin=Origin(
                xyz=(-axis_x * 0.0033, 0.000, joint_z - axis_z * 0.0033),
                rpy=(0.000, pitch, 0.000),
            ),
            material=bushing_smoke,
            name="lower_bushing",
        )
        baseplate.visual(
            Cylinder(radius=0.0058, length=0.0024),
            origin=Origin(
                xyz=(axis_x * 0.0067, 0.000, joint_z + axis_z * 0.0067),
                rpy=(0.000, pitch, 0.000),
            ),
            material=hardware_dark,
            name="kingpin_top_nut",
        )
        baseplate.inertial = Inertial.from_geometry(
            Box((0.080, 0.056, 0.036)),
            mass=0.18,
            origin=Origin(xyz=(0.000, 0.000, -0.010)),
        )

        truck = model.part(f"{prefix}_truck")
        truck.visual(
            pivot_sleeve_mesh,
            origin=Origin(xyz=(0.000, 0.000, joint_z), rpy=(0.000, pitch, 0.000)),
            material=cast_aluminum,
            name="pivot_sleeve",
        )
        truck.visual(
            Box((0.022, 0.026, 0.010)),
            origin=Origin(xyz=(0.005 * centerward_sign, 0.000, -0.014)),
            material=cast_aluminum,
            name="hanger_yoke",
        )
        truck.visual(
            Box((0.016, 0.024, 0.022)),
            origin=Origin(xyz=(0.002 * centerward_sign, 0.000, -0.024)),
            material=cast_aluminum,
            name="hanger_web",
        )
        truck.visual(
            Box((0.012, 0.020, 0.009)),
            origin=Origin(xyz=(0.001 * centerward_sign, 0.000, -0.0330)),
            material=cast_aluminum,
            name="hanger_drop",
        )
        truck.visual(
            Box((0.014, 0.132, 0.012)),
            origin=Origin(xyz=(0.000, 0.000, axle_z)),
            material=cast_aluminum,
            name="hanger_body",
        )
        truck.visual(
            Box((0.012, 0.008, 0.012)),
            origin=Origin(xyz=(0.000, 0.062, axle_z)),
            material=cast_aluminum,
            name="left_axle_boss",
        )
        truck.visual(
            Box((0.012, 0.008, 0.012)),
            origin=Origin(xyz=(0.000, -0.062, axle_z)),
            material=cast_aluminum,
            name="right_axle_boss",
        )
        truck.visual(
            Cylinder(radius=0.0024, length=0.196),
            origin=Origin(xyz=(0.000, 0.000, axle_z), rpy=(pi / 2.0, 0.000, 0.000)),
            material=satin_steel,
            name="axle",
        )
        truck.visual(
            Cylinder(radius=0.0066, length=0.0020),
            origin=Origin(xyz=(0.000, wheel_inner_face - 0.0010, axle_z), rpy=(pi / 2.0, 0.000, 0.000)),
            material=satin_steel,
            name="left_inner_washer",
        )
        truck.visual(
            Cylinder(radius=0.0066, length=0.0020),
            origin=Origin(xyz=(0.000, -(wheel_inner_face - 0.0010), axle_z), rpy=(pi / 2.0, 0.000, 0.000)),
            material=satin_steel,
            name="right_inner_washer",
        )
        truck.inertial = Inertial.from_geometry(
            Box((0.090, 0.196, 0.050)),
            mass=0.34,
            origin=Origin(xyz=(0.000, 0.000, -0.022)),
        )

        model.articulation(
            f"deck_to_{prefix}_baseplate",
            ArticulationType.FIXED,
            parent=deck,
            child=baseplate,
            origin=Origin(xyz=(x_pos, 0.000, baseplate_mount_z)),
        )
        model.articulation(
            f"{prefix}_pivot",
            ArticulationType.REVOLUTE,
            parent=baseplate,
            child=truck,
            origin=Origin(xyz=(0.000, 0.000, joint_z)),
            axis=(axis_x, 0.000, axis_z),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=2.4,
                lower=-0.16,
                upper=0.16,
            ),
        )

        for side_name, side_y in (("left", wheel_center_y), ("right", -wheel_center_y)):
            wheel = model.part(f"{prefix}_{side_name}_wheel")
            wheel.visual(
                wheel_outer_mesh,
                origin=Origin(rpy=(pi / 2.0, 0.000, 0.000)),
                material=wheel_urethane,
                name="wheel_tread",
            )
            wheel.visual(
                wheel_core_mesh,
                origin=Origin(rpy=(pi / 2.0, 0.000, 0.000)),
                material=wheel_core,
                name="wheel_core",
            )
            wheel.inertial = Inertial.from_geometry(
                Cylinder(radius=0.0265, length=0.031),
                mass=0.090,
                origin=Origin(rpy=(pi / 2.0, 0.000, 0.000)),
            )
            model.articulation(
                f"{prefix}_{side_name}_wheel_spin",
                ArticulationType.CONTINUOUS,
                parent=truck,
                child=wheel,
                origin=Origin(xyz=(0.000, side_y, axle_z)),
                axis=(0.000, 1.000, 0.000),
                motion_limits=MotionLimits(effort=2.5, velocity=45.0),
            )

    add_truck("rear", x_pos=-0.182, pitch=0.58, centerward_sign=1.0)
    add_truck("front", x_pos=0.182, pitch=-0.58, centerward_sign=-1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    deck = object_model.get_part("deck")
    front_baseplate = object_model.get_part("front_baseplate")
    rear_baseplate = object_model.get_part("rear_baseplate")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_pivot = object_model.get_articulation("front_pivot")
    rear_pivot = object_model.get_articulation("rear_pivot")
    front_left_spin = object_model.get_articulation("front_left_wheel_spin")
    rear_right_spin = object_model.get_articulation("rear_right_wheel_spin")

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

    ctx.expect_contact(deck, front_baseplate, name="front_baseplate_contacts_deck")
    ctx.expect_contact(deck, rear_baseplate, name="rear_baseplate_contacts_deck")
    ctx.expect_contact(front_baseplate, front_truck, name="front_truck_supported_by_bushings")
    ctx.expect_contact(rear_baseplate, rear_truck, name="rear_truck_supported_by_bushings")

    ctx.expect_contact(front_left_wheel, front_truck, name="front_left_wheel_on_axle")
    ctx.expect_contact(front_right_wheel, front_truck, name="front_right_wheel_on_axle")
    ctx.expect_contact(rear_left_wheel, rear_truck, name="rear_left_wheel_on_axle")
    ctx.expect_contact(rear_right_wheel, rear_truck, name="rear_right_wheel_on_axle")

    ctx.expect_origin_distance(
        front_baseplate,
        rear_baseplate,
        axes="x",
        min_dist=0.34,
        max_dist=0.39,
        name="realistic_wheelbase",
    )
    ctx.expect_origin_distance(
        front_left_wheel,
        front_right_wheel,
        axes="y",
        min_dist=0.15,
        max_dist=0.18,
        name="realistic_front_track_width",
    )
    ctx.expect_origin_distance(
        rear_left_wheel,
        rear_right_wheel,
        axes="y",
        min_dist=0.15,
        max_dist=0.18,
        name="realistic_rear_track_width",
    )
    ctx.expect_origin_gap(
        deck,
        front_left_wheel,
        axis="z",
        min_gap=0.03,
        max_gap=0.06,
        name="front_wheels_hang_below_deck",
    )
    ctx.expect_origin_gap(
        deck,
        rear_left_wheel,
        axis="z",
        min_gap=0.03,
        max_gap=0.06,
        name="rear_wheels_hang_below_deck",
    )

    front_limits = front_pivot.motion_limits
    rear_limits = rear_pivot.motion_limits
    if front_limits is not None and front_limits.lower is not None and front_limits.upper is not None:
        with ctx.pose({front_pivot: front_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="front_pivot_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="front_pivot_lower_no_floating")
            ctx.expect_contact(front_baseplate, front_truck, name="front_pivot_lower_contact")
        with ctx.pose({front_pivot: front_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="front_pivot_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="front_pivot_upper_no_floating")
            ctx.expect_contact(front_baseplate, front_truck, name="front_pivot_upper_contact")

    if rear_limits is not None and rear_limits.lower is not None and rear_limits.upper is not None:
        with ctx.pose({rear_pivot: rear_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_pivot_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_pivot_lower_no_floating")
            ctx.expect_contact(rear_baseplate, rear_truck, name="rear_pivot_lower_contact")
        with ctx.pose({rear_pivot: rear_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_pivot_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_pivot_upper_no_floating")
            ctx.expect_contact(rear_baseplate, rear_truck, name="rear_pivot_upper_contact")

    with ctx.pose({front_left_spin: pi * 0.75, rear_right_spin: pi * 1.10}):
        ctx.expect_contact(front_left_wheel, front_truck, name="front_left_wheel_contact_while_spinning")
        ctx.expect_contact(rear_right_wheel, rear_truck, name="rear_right_wheel_contact_while_spinning")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
