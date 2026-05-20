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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_skateboard", assets=ASSETS)

    deck_paint = model.material("deck_paint", rgba=(0.20, 0.24, 0.21, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    edge_bumper = model.material("edge_bumper", rgba=(0.08, 0.08, 0.09, 1.0))
    truck_paint = model.material("truck_paint", rgba=(0.32, 0.34, 0.36, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    hardware = model.material("hardware", rgba=(0.72, 0.74, 0.77, 1.0))
    bushing_urethane = model.material("bushing_urethane", rgba=(0.84, 0.47, 0.16, 1.0))
    wheel_poly = model.material("wheel_poly", rgba=(0.78, 0.62, 0.28, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.16, 0.16, 0.17, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def _deck_loop(
        x_pos: float,
        *,
        width: float,
        top_z: float,
        bottom_z: float,
        dish: float,
        rail_lift: float,
    ) -> list[tuple[float, float, float]]:
        half_width = width * 0.5
        return [
            (x_pos, 0.0, top_z - dish),
            (x_pos, half_width * 0.30, top_z),
            (x_pos, half_width * 0.76, top_z + rail_lift),
            (x_pos, half_width, top_z + rail_lift * 0.55 - 0.002),
            (x_pos, half_width, bottom_z + 0.004),
            (x_pos, half_width * 0.34, bottom_z),
            (x_pos, 0.0, bottom_z - 0.001),
            (x_pos, -half_width * 0.34, bottom_z),
            (x_pos, -half_width, bottom_z + 0.004),
            (x_pos, -half_width, top_z + rail_lift * 0.55 - 0.002),
            (x_pos, -half_width * 0.76, top_z + rail_lift),
            (x_pos, -half_width * 0.30, top_z),
        ]

    def _deck_mesh():
        sections = [
            _deck_loop(-0.440, width=0.150, top_z=0.032, bottom_z=0.014, dish=0.001, rail_lift=0.003),
            _deck_loop(-0.345, width=0.212, top_z=0.020, bottom_z=0.001, dish=0.0015, rail_lift=0.004),
            _deck_loop(-0.220, width=0.248, top_z=0.012, bottom_z=-0.009, dish=0.0022, rail_lift=0.006),
            _deck_loop(0.000, width=0.255, top_z=0.0115, bottom_z=-0.0095, dish=0.0025, rail_lift=0.0065),
            _deck_loop(0.220, width=0.248, top_z=0.012, bottom_z=-0.009, dish=0.0022, rail_lift=0.006),
            _deck_loop(0.345, width=0.212, top_z=0.020, bottom_z=0.001, dish=0.0015, rail_lift=0.004),
            _deck_loop(0.440, width=0.150, top_z=0.032, bottom_z=0.014, dish=0.001, rail_lift=0.003),
        ]
        return section_loft(sections)

    def _yz_loop(x_pos: float, *, width: float, height: float, center_z: float) -> list[tuple[float, float, float]]:
        half_width = width * 0.5
        half_height = height * 0.5
        return [
            (x_pos, 0.0, center_z + half_height),
            (x_pos, half_width * 0.68, center_z + half_height * 0.84),
            (x_pos, half_width, center_z + half_height * 0.18),
            (x_pos, half_width * 0.90, center_z - half_height * 0.72),
            (x_pos, 0.0, center_z - half_height),
            (x_pos, -half_width * 0.90, center_z - half_height * 0.72),
            (x_pos, -half_width, center_z + half_height * 0.18),
            (x_pos, -half_width * 0.68, center_z + half_height * 0.84),
        ]

    def _xz_loop(y_pos: float, *, width: float, height: float, center_z: float) -> list[tuple[float, float, float]]:
        half_width = width * 0.5
        half_height = height * 0.5
        return [
            (0.0, y_pos, center_z + half_height),
            (half_width * 0.72, y_pos, center_z + half_height * 0.86),
            (half_width, y_pos, center_z + half_height * 0.12),
            (half_width * 0.88, y_pos, center_z - half_height * 0.74),
            (0.0, y_pos, center_z - half_height),
            (-half_width * 0.88, y_pos, center_z - half_height * 0.74),
            (-half_width, y_pos, center_z + half_height * 0.12),
            (-half_width * 0.72, y_pos, center_z + half_height * 0.86),
        ]

    def _wheel_shell_mesh(name: str, *, radius: float, width: float, bore_radius: float):
        half_width = width * 0.5
        geometry = LatheGeometry.from_shell_profiles(
            [
                (0.020, -half_width),
                (0.030, -half_width),
                (radius * 0.95, -half_width * 0.70),
                (radius, -half_width * 0.22),
                (radius, half_width * 0.22),
                (radius * 0.95, half_width * 0.70),
                (0.030, half_width),
                (0.020, half_width),
            ],
            [
                (bore_radius, -half_width),
                (bore_radius, half_width),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(math.pi / 2.0)
        return _save_mesh(name, geometry)

    def _wheel_core_mesh(name: str, *, width: float, outer_radius: float, inner_radius: float):
        half_width = width * 0.5
        geometry = LatheGeometry.from_shell_profiles(
            [
                (outer_radius * 0.90, -half_width * 0.60),
                (outer_radius, -half_width * 0.48),
                (outer_radius, half_width * 0.48),
                (outer_radius * 0.90, half_width * 0.60),
            ],
            [
                (inner_radius, -half_width * 0.60),
                (inner_radius, half_width * 0.60),
            ],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(math.pi / 2.0)
        return _save_mesh(name, geometry)

    wheel_shell_mesh = _wheel_shell_mesh(
        "utility_skateboard_wheel_shell.obj",
        radius=0.037,
        width=0.040,
        bore_radius=0.0045,
    )
    baseplate_shell_mesh = _save_mesh(
        "utility_skateboard_baseplate_shell.obj",
        section_loft(
            [
                _yz_loop(-0.048, width=0.044, height=0.010, center_z=0.006),
                _yz_loop(-0.012, width=0.062, height=0.016, center_z=0.009),
                _yz_loop(0.020, width=0.058, height=0.020, center_z=0.011),
                _yz_loop(0.048, width=0.032, height=0.014, center_z=0.012),
            ]
        ),
    )
    hanger_shell_mesh = _save_mesh(
        "utility_skateboard_hanger_shell.obj",
        section_loft(
            [
                _xz_loop(-0.106, width=0.024, height=0.018, center_z=-0.032),
                _xz_loop(-0.074, width=0.040, height=0.026, center_z=-0.031),
                _xz_loop(-0.034, width=0.060, height=0.034, center_z=-0.025),
                _xz_loop(0.000, width=0.074, height=0.038, center_z=-0.020),
                _xz_loop(0.034, width=0.060, height=0.034, center_z=-0.025),
                _xz_loop(0.074, width=0.040, height=0.026, center_z=-0.031),
                _xz_loop(0.106, width=0.024, height=0.018, center_z=-0.032),
            ]
        ),
    )

    deck = model.part("deck")
    deck.visual(
        _save_mesh("utility_skateboard_deck.obj", _deck_mesh()),
        material=deck_paint,
        name="deck_shell",
    )
    deck.visual(
        Box((0.640, 0.132, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.0115)),
        material=grip_rubber,
        name="traction_pad",
    )
    deck.visual(
        Box((0.170, 0.110, 0.003)),
        origin=Origin(xyz=(0.312, 0.000, 0.0185)),
        material=grip_rubber,
        name="nose_patch",
    )
    deck.visual(
        Box((0.170, 0.110, 0.003)),
        origin=Origin(xyz=(-0.312, 0.000, 0.0185)),
        material=grip_rubber,
        name="tail_patch",
    )
    deck.visual(
        Box((0.720, 0.014, 0.018)),
        origin=Origin(xyz=(0.000, 0.117, -0.004)),
        material=edge_bumper,
        name="left_rail_guard",
    )
    deck.visual(
        Box((0.720, 0.014, 0.018)),
        origin=Origin(xyz=(0.000, -0.117, -0.004)),
        material=edge_bumper,
        name="right_rail_guard",
    )
    deck.visual(
        Box((0.090, 0.110, 0.020)),
        origin=Origin(xyz=(0.400, 0.000, 0.020)),
        material=edge_bumper,
        name="nose_bash_block",
    )
    deck.visual(
        Box((0.090, 0.110, 0.020)),
        origin=Origin(xyz=(-0.400, 0.000, 0.020)),
        material=edge_bumper,
        name="tail_bash_block",
    )
    deck.visual(
        Box((0.132, 0.100, 0.012)),
        origin=Origin(xyz=(0.220, 0.000, -0.013)),
        material=deck_paint,
        name="front_mount_block",
    )
    deck.visual(
        Box((0.132, 0.100, 0.012)),
        origin=Origin(xyz=(-0.220, 0.000, -0.013)),
        material=deck_paint,
        name="rear_mount_block",
    )
    for mount_name, mount_x in (("front", 0.220), ("rear", -0.220)):
        for x_offset in (-0.022, 0.022):
            for y_offset in (-0.028, 0.028):
                deck.visual(
                    Cylinder(radius=0.0042, length=0.003),
                    origin=Origin(xyz=(mount_x + x_offset, y_offset, 0.0122)),
                    material=hardware,
                    name=f"{mount_name}_bolt_{'p' if x_offset > 0 else 'm'}{'p' if y_offset > 0 else 'm'}",
                )
    deck.inertial = Inertial.from_geometry(
        Box((0.880, 0.255, 0.060)),
        mass=3.8,
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
    )

    def _build_baseplate(name: str):
        baseplate = model.part(name)
        baseplate.visual(
            baseplate_shell_mesh,
            material=truck_paint,
            name="baseplate_shell",
        )
        baseplate.visual(
            Box((0.096, 0.060, 0.020)),
            origin=Origin(xyz=(0.000, 0.000, 0.010)),
            material=truck_paint,
            name="mount_pad",
        )
        baseplate.visual(
            Cylinder(radius=0.028, length=0.006),
            origin=Origin(xyz=(0.000, 0.000, 0.003)),
            material=truck_paint,
            name="swivel_pad",
        )
        baseplate.visual(
            Cylinder(radius=0.014, length=0.011),
            origin=Origin(xyz=(0.000, 0.000, 0.0115)),
            material=bushing_urethane,
            name="lower_bushing",
        )
        baseplate.visual(
            Cylinder(radius=0.018, length=0.003),
            origin=Origin(xyz=(0.000, 0.000, 0.0185)),
            material=hardware,
            name="lower_washer",
        )
        baseplate.visual(
            Box((0.040, 0.034, 0.016)),
            origin=Origin(xyz=(0.020, 0.000, 0.008)),
            material=truck_paint,
            name="pivot_house",
        )
        for x_offset in (-0.022, 0.022):
            for y_offset in (-0.028, 0.028):
                baseplate.visual(
                    Cylinder(radius=0.0045, length=0.008),
                    origin=Origin(xyz=(x_offset, y_offset, 0.006)),
                    material=hardware,
                    name=f"nut_{'p' if x_offset > 0 else 'm'}{'p' if y_offset > 0 else 'm'}",
                )
        baseplate.inertial = Inertial.from_geometry(
            Box((0.096, 0.060, 0.024)),
            mass=0.45,
            origin=Origin(xyz=(0.000, 0.000, 0.010)),
        )
        return baseplate

    def _build_hanger(name: str):
        hanger = model.part(name)
        hanger.visual(
            hanger_shell_mesh,
            material=truck_paint,
            name="hanger_shell",
        )
        hanger.visual(
            Cylinder(radius=0.028, length=0.006),
            origin=Origin(xyz=(0.000, 0.000, -0.003)),
            material=truck_paint,
            name="turntable_pad",
        )
        hanger.visual(
            Box((0.060, 0.156, 0.024)),
            origin=Origin(xyz=(0.000, 0.000, -0.017)),
            material=truck_paint,
            name="hanger_body",
        )
        hanger.visual(
            Box((0.036, 0.192, 0.016)),
            origin=Origin(xyz=(0.000, 0.000, -0.030)),
            material=truck_paint,
            name="axle_tie_bar",
        )
        hanger.visual(
            Box((0.040, 0.028, 0.026)),
            origin=Origin(xyz=(0.000, 0.084, -0.033)),
            material=truck_paint,
            name="left_axle_seat",
        )
        hanger.visual(
            Box((0.040, 0.028, 0.026)),
            origin=Origin(xyz=(0.000, -0.084, -0.033)),
            material=truck_paint,
            name="right_axle_seat",
        )
        hanger.visual(
            Box((0.044, 0.030, 0.018)),
            origin=Origin(xyz=(0.024, 0.000, -0.016)),
            material=truck_paint,
            name="pivot_anchor",
        )
        hanger.visual(
            Cylinder(radius=0.0115, length=0.010),
            origin=Origin(xyz=(0.000, 0.000, -0.008)),
            material=bushing_urethane,
            name="upper_bushing",
        )
        hanger.visual(
            Cylinder(radius=0.016, length=0.002),
            origin=Origin(xyz=(0.000, 0.000, -0.002)),
            material=hardware,
            name="upper_washer",
        )
        hanger.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(0.000, 0.000, -0.016)),
            material=hardware,
            name="kingpin_nut",
        )
        hanger.inertial = Inertial.from_geometry(
            Box((0.060, 0.210, 0.042)),
            mass=0.72,
            origin=Origin(xyz=(0.000, 0.000, -0.022)),
        )
        return hanger

    def _build_axle(name: str, *, side_sign: float):
        axle = model.part(name)
        axle.visual(
            Box((0.022, 0.018, 0.018)),
            origin=Origin(xyz=(0.000, side_sign * 0.009, 0.000)),
            material=truck_paint,
            name="inner_block",
        )
        axle.visual(
            Cylinder(radius=0.0045, length=0.076),
            origin=Origin(xyz=(0.000, side_sign * 0.053, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=axle_steel,
            name="axle_shaft",
        )
        axle.visual(
            Cylinder(radius=0.0075, length=0.008),
            origin=Origin(xyz=(0.000, side_sign * 0.030, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name="inner_spacer",
        )
        axle.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(0.000, side_sign * 0.091, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name="outer_nut",
        )
        axle.inertial = Inertial.from_geometry(
            Box((0.022, 0.094, 0.018)),
            mass=0.10,
            origin=Origin(xyz=(0.000, side_sign * 0.045, 0.000)),
        )
        return axle

    def _build_wheel(name: str):
        wheel = model.part(name)
        wheel.visual(
            wheel_shell_mesh,
            material=wheel_poly,
            name="wheel_shell",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.037, length=0.042),
            mass=0.36,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        return wheel

    front_baseplate = _build_baseplate("front_baseplate")
    rear_baseplate = _build_baseplate("rear_baseplate")
    front_hanger = _build_hanger("front_hanger")
    rear_hanger = _build_hanger("rear_hanger")

    front_left_axle = _build_axle("front_left_axle", side_sign=1.0)
    front_right_axle = _build_axle("front_right_axle", side_sign=-1.0)
    rear_left_axle = _build_axle("rear_left_axle", side_sign=1.0)
    rear_right_axle = _build_axle("rear_right_axle", side_sign=-1.0)

    front_left_wheel = _build_wheel("front_left_wheel")
    front_right_wheel = _build_wheel("front_right_wheel")
    rear_left_wheel = _build_wheel("rear_left_wheel")
    rear_right_wheel = _build_wheel("rear_right_wheel")

    model.articulation(
        "deck_to_front_baseplate",
        ArticulationType.FIXED,
        parent=deck,
        child=front_baseplate,
        origin=Origin(xyz=(0.220, 0.000, -0.039)),
    )
    model.articulation(
        "deck_to_rear_baseplate",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_baseplate,
        origin=Origin(xyz=(-0.220, 0.000, -0.039)),
    )
    model.articulation(
        "front_truck_pivot",
        ArticulationType.REVOLUTE,
        parent=front_baseplate,
        child=front_hanger,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-0.34,
            upper=0.34,
        ),
    )
    model.articulation(
        "rear_truck_pivot",
        ArticulationType.REVOLUTE,
        parent=rear_baseplate,
        child=rear_hanger,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-0.34,
            upper=0.34,
        ),
    )

    for hanger_name, hanger_part, left_axle, right_axle in (
        ("front", front_hanger, front_left_axle, front_right_axle),
        ("rear", rear_hanger, rear_left_axle, rear_right_axle),
    ):
        model.articulation(
            f"{hanger_name}_hanger_to_left_axle",
            ArticulationType.FIXED,
            parent=hanger_part,
            child=left_axle,
            origin=Origin(xyz=(0.000, 0.098, -0.034)),
        )
        model.articulation(
            f"{hanger_name}_hanger_to_right_axle",
            ArticulationType.FIXED,
            parent=hanger_part,
            child=right_axle,
            origin=Origin(xyz=(0.000, -0.098, -0.034)),
        )

    wheel_specs = (
        ("front_left_wheel_spin", front_left_axle, front_left_wheel, 1.0),
        ("front_right_wheel_spin", front_right_axle, front_right_wheel, -1.0),
        ("rear_left_wheel_spin", rear_left_axle, rear_left_wheel, 1.0),
        ("rear_right_wheel_spin", rear_right_axle, rear_right_wheel, -1.0),
    )
    for joint_name, axle_part, wheel_part, side_sign in wheel_specs:
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=axle_part,
            child=wheel_part,
            origin=Origin(xyz=(0.000, side_sign * 0.062, 0.000)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=40.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    deck = object_model.get_part("deck")
    front_baseplate = object_model.get_part("front_baseplate")
    rear_baseplate = object_model.get_part("rear_baseplate")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_left_axle = object_model.get_part("front_left_axle")
    front_right_axle = object_model.get_part("front_right_axle")
    rear_left_axle = object_model.get_part("rear_left_axle")
    rear_right_axle = object_model.get_part("rear_right_axle")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    front_truck_pivot = object_model.get_articulation("front_truck_pivot")
    rear_truck_pivot = object_model.get_articulation("rear_truck_pivot")
    front_left_wheel_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_wheel_spin = object_model.get_articulation("front_right_wheel_spin")
    rear_left_wheel_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_wheel_spin = object_model.get_articulation("rear_right_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        front_hanger,
        front_left_axle,
        reason="Left front axle is modeled as a keyed serviceable stub captured inside the reinforced hanger socket.",
    )
    ctx.allow_overlap(
        front_hanger,
        front_right_axle,
        reason="Right front axle is modeled as a keyed serviceable stub captured inside the reinforced hanger socket.",
    )
    ctx.allow_overlap(
        rear_hanger,
        rear_left_axle,
        reason="Left rear axle is modeled as a keyed serviceable stub captured inside the reinforced hanger socket.",
    )
    ctx.allow_overlap(
        rear_hanger,
        rear_right_axle,
        reason="Right rear axle is modeled as a keyed serviceable stub captured inside the reinforced hanger socket.",
    )
    ctx.allow_overlap(
        front_left_axle,
        front_left_wheel,
        reason="The wheel visual is authored as a sealed utility wheel shell around hidden bearings, so the axle spindle occupies concealed internal hub volume.",
    )
    ctx.allow_overlap(
        front_right_axle,
        front_right_wheel,
        reason="The wheel visual is authored as a sealed utility wheel shell around hidden bearings, so the axle spindle occupies concealed internal hub volume.",
    )
    ctx.allow_overlap(
        rear_left_axle,
        rear_left_wheel,
        reason="The wheel visual is authored as a sealed utility wheel shell around hidden bearings, so the axle spindle occupies concealed internal hub volume.",
    )
    ctx.allow_overlap(
        rear_right_axle,
        rear_right_wheel,
        reason="The wheel visual is authored as a sealed utility wheel shell around hidden bearings, so the axle spindle occupies concealed internal hub volume.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.check(
        "front_truck_axis_is_vertical",
        front_truck_pivot.axis == (0.0, 0.0, 1.0),
        details=f"front_truck_pivot axis was {front_truck_pivot.axis}",
    )
    ctx.check(
        "rear_truck_axis_is_vertical",
        rear_truck_pivot.axis == (0.0, 0.0, 1.0),
        details=f"rear_truck_pivot axis was {rear_truck_pivot.axis}",
    )
    for joint_name, joint in (
        ("front_left_wheel_spin", front_left_wheel_spin),
        ("front_right_wheel_spin", front_right_wheel_spin),
        ("rear_left_wheel_spin", rear_left_wheel_spin),
        ("rear_right_wheel_spin", rear_right_wheel_spin),
    ):
        ctx.check(
            f"{joint_name}_axis_is_axial",
            joint.axis == (0.0, 1.0, 0.0),
            details=f"{joint_name} axis was {joint.axis}",
        )

    ctx.expect_contact(front_baseplate, deck)
    ctx.expect_contact(rear_baseplate, deck)
    ctx.expect_overlap(front_baseplate, deck, axes="xy", min_overlap=0.05)
    ctx.expect_overlap(rear_baseplate, deck, axes="xy", min_overlap=0.05)

    ctx.expect_contact(front_hanger, front_baseplate, elem_a="turntable_pad", elem_b="swivel_pad")
    ctx.expect_contact(rear_hanger, rear_baseplate, elem_a="turntable_pad", elem_b="swivel_pad")

    ctx.expect_contact(front_left_axle, front_hanger, elem_a="inner_block", elem_b="left_axle_seat")
    ctx.expect_contact(front_right_axle, front_hanger, elem_a="inner_block", elem_b="right_axle_seat")
    ctx.expect_contact(rear_left_axle, rear_hanger, elem_a="inner_block", elem_b="left_axle_seat")
    ctx.expect_contact(rear_right_axle, rear_hanger, elem_a="inner_block", elem_b="right_axle_seat")

    ctx.expect_contact(front_left_wheel, front_left_axle)
    ctx.expect_contact(front_right_wheel, front_right_axle)
    ctx.expect_contact(rear_left_wheel, rear_left_axle)
    ctx.expect_contact(rear_right_wheel, rear_right_axle)

    ctx.expect_origin_distance(front_baseplate, rear_baseplate, axes="x", min_dist=0.42, max_dist=0.46)
    ctx.expect_origin_distance(front_left_wheel, front_right_wheel, axes="y", min_dist=0.28, max_dist=0.34)
    ctx.expect_origin_distance(rear_left_wheel, rear_right_wheel, axes="y", min_dist=0.28, max_dist=0.34)

    ctx.expect_gap(deck, front_left_wheel, axis="z", min_gap=0.014, max_gap=0.040)
    ctx.expect_gap(deck, front_right_wheel, axis="z", min_gap=0.014, max_gap=0.040)
    ctx.expect_gap(deck, rear_left_wheel, axis="z", min_gap=0.014, max_gap=0.040)
    ctx.expect_gap(deck, rear_right_wheel, axis="z", min_gap=0.014, max_gap=0.040)

    deck_aabb = ctx.part_world_aabb(deck)
    if deck_aabb is None:
        ctx.fail("deck_aabb_available", "Deck AABB was not available.")
    else:
        deck_length = deck_aabb[1][0] - deck_aabb[0][0]
        deck_width = deck_aabb[1][1] - deck_aabb[0][1]
        ctx.check(
            "deck_length_realistic",
            0.84 <= deck_length <= 0.90,
            details=f"Deck length was {deck_length:.4f} m",
        )
        ctx.check(
            "deck_width_realistic",
            0.24 <= deck_width <= 0.27,
            details=f"Deck width was {deck_width:.4f} m",
        )

    for pose_name, truck_angle in (("left_stop", -0.34), ("right_stop", 0.34)):
        with ctx.pose({front_truck_pivot: truck_angle, rear_truck_pivot: truck_angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{pose_name}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{pose_name}_no_floating")
            ctx.expect_contact(
                front_hanger,
                front_baseplate,
                elem_a="turntable_pad",
                elem_b="swivel_pad",
                name=f"{pose_name}_front_pivot_contact",
            )
            ctx.expect_contact(
                rear_hanger,
                rear_baseplate,
                elem_a="turntable_pad",
                elem_b="swivel_pad",
                name=f"{pose_name}_rear_pivot_contact",
            )

    with ctx.pose(
        {
            front_left_wheel_spin: 1.8,
            front_right_wheel_spin: -0.9,
            rear_left_wheel_spin: 2.6,
            rear_right_wheel_spin: -1.4,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_spin_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_spin_pose_no_floating")
        ctx.expect_contact(front_left_wheel, front_left_axle, name="front_left_wheel_spin_contact")
        ctx.expect_contact(rear_right_wheel, rear_right_axle, name="rear_right_wheel_spin_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
