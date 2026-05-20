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
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_skateboard")

    deck_composite = model.material("deck_composite", rgba=(0.18, 0.19, 0.20, 1.0))
    mount_seal = model.material("mount_seal", rgba=(0.10, 0.10, 0.11, 1.0))
    hanger_metal = model.material("hanger_metal", rgba=(0.63, 0.66, 0.70, 1.0))
    baseplate_metal = model.material("baseplate_metal", rgba=(0.48, 0.51, 0.55, 1.0))
    stainless = model.material("stainless", rgba=(0.79, 0.81, 0.84, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.42, 0.49, 0.44, 1.0))
    bushing_urethane = model.material("bushing_urethane", rgba=(0.79, 0.70, 0.28, 1.0))

    deck_length = 0.84
    deck_width = 0.238
    deck_thickness = 0.016
    truck_spacing = 0.46
    truck_x = truck_spacing * 0.5
    kingpin_pitch = math.radians(32.0)

    def _deck_loop(
        x_pos: float,
        width: float,
        thickness: float,
        center_z: float,
        top_crown: float,
    ) -> list[tuple[float, float, float]]:
        half_w = width * 0.5
        top_z = center_z + thickness * 0.5
        bottom_z = center_z - thickness * 0.5
        return [
            (x_pos, half_w, bottom_z + 0.0012),
            (x_pos, half_w * 0.97, center_z - 0.0010),
            (x_pos, half_w * 0.93, top_z - 0.0014),
            (x_pos, half_w * 0.64, top_z - 0.0005),
            (x_pos, half_w * 0.24, top_z - top_crown * 0.82),
            (x_pos, 0.0, top_z - top_crown),
            (x_pos, -half_w * 0.24, top_z - top_crown * 0.82),
            (x_pos, -half_w * 0.64, top_z - 0.0005),
            (x_pos, -half_w * 0.93, top_z - 0.0014),
            (x_pos, -half_w * 0.97, center_z - 0.0010),
            (x_pos, -half_w, bottom_z + 0.0012),
            (x_pos, -half_w * 0.62, bottom_z + 0.0006),
            (x_pos, -half_w * 0.18, bottom_z + 0.0018),
            (x_pos, half_w * 0.18, bottom_z + 0.0018),
            (x_pos, half_w * 0.62, bottom_z + 0.0006),
        ]

    def _axis_offset(distance: float) -> tuple[float, float, float]:
        return (
            distance * math.cos(kingpin_pitch),
            0.0,
            -distance * math.sin(kingpin_pitch),
        )

    deck_sections = [
        _deck_loop(-0.420, 0.170, 0.012, 0.030, 0.0010),
        _deck_loop(-0.360, 0.194, 0.014, 0.018, 0.0014),
        _deck_loop(-0.280, 0.220, 0.015, 0.007, 0.0018),
        _deck_loop(-0.180, 0.232, 0.016, 0.001, 0.0021),
        _deck_loop(-0.060, deck_width, deck_thickness, 0.000, 0.0025),
        _deck_loop(0.060, deck_width, deck_thickness, 0.000, 0.0025),
        _deck_loop(0.180, 0.232, 0.016, 0.001, 0.0021),
        _deck_loop(0.280, 0.220, 0.015, 0.007, 0.0018),
        _deck_loop(0.360, 0.194, 0.014, 0.018, 0.0014),
        _deck_loop(0.420, 0.170, 0.012, 0.030, 0.0010),
    ]
    deck_mesh = mesh_from_geometry(section_loft(deck_sections), "weatherproof_skateboard_deck")

    wheel_profile = [
        (0.011, -0.022),
        (0.019, -0.025),
        (0.028, -0.022),
        (0.032, -0.013),
        (0.033, -0.004),
        (0.033, 0.004),
        (0.032, 0.013),
        (0.028, 0.022),
        (0.019, 0.025),
        (0.011, 0.022),
        (0.011, 0.010),
        (0.014, 0.005),
        (0.014, -0.005),
        (0.011, -0.010),
    ]
    wheel_mesh = mesh_from_geometry(
        LatheGeometry(wheel_profile, segments=56).rotate_x(-math.pi / 2.0),
        "weatherproof_skateboard_wheel",
    )

    deck = model.part("deck")
    deck.visual(deck_mesh, material=deck_composite, name="deck_shell")
    deck.visual(
        Box((0.110, 0.078, 0.004)),
        origin=Origin(xyz=(truck_x, 0.0, -0.010)),
        material=mount_seal,
        name="front_mount_pad",
    )
    deck.visual(
        Box((0.110, 0.078, 0.004)),
        origin=Origin(xyz=(-truck_x, 0.0, -0.010)),
        material=mount_seal,
        name="rear_mount_pad",
    )
    for truck_sign, truck_center_x in ((1.0, truck_x), (-1.0, -truck_x)):
        for x_off in (-0.022, 0.022):
            for y_off in (-0.020, 0.020):
                bolt_name = (
                    f"{'front' if truck_sign > 0.0 else 'rear'}_bolt_{'l' if x_off < 0.0 else 'r'}"
                    f"{'n' if y_off > 0.0 else 's'}"
                )
                deck.visual(
                    Cylinder(radius=0.0026, length=0.021),
                    origin=Origin(xyz=(truck_center_x + x_off, y_off, -0.0015)),
                    material=stainless,
                    name=f"{bolt_name}_shaft",
                )
                deck.visual(
                    Cylinder(radius=0.0048, length=0.003),
                    origin=Origin(xyz=(truck_center_x + x_off, y_off, 0.0105)),
                    material=stainless,
                    name=f"{bolt_name}_head",
                )
    deck.inertial = Inertial.from_geometry(
        Box((deck_length, deck_width, 0.040)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    def _build_baseplate(part_name: str) -> object:
        part = model.part(part_name)
        part.visual(
            Box((0.108, 0.078, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=mount_seal,
            name="gasket_pad",
        )
        part.visual(
            Box((0.096, 0.068, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=baseplate_metal,
            name="mount_plate",
        )
        part.visual(
            Box((0.024, 0.042, 0.020)),
            origin=Origin(xyz=(0.030, 0.0, -0.018)),
            material=baseplate_metal,
            name="kingpin_tower",
        )
        part.visual(
            Box((0.026, 0.050, 0.006)),
            origin=Origin(xyz=(0.034, 0.0, -0.009)),
            material=baseplate_metal,
            name="spray_shroud",
        )
        for cheek_sign in (-1.0, 1.0):
            part.visual(
                Box((0.024, 0.010, 0.018)),
                origin=Origin(xyz=(0.026, cheek_sign * 0.026, -0.017)),
                material=baseplate_metal,
                name=f"{'left' if cheek_sign > 0.0 else 'right'}_cheek",
            )
        axis_rpy = (0.0, kingpin_pitch + math.pi / 2.0, 0.0)
        lower_offset = _axis_offset(-0.012)
        upper_offset = _axis_offset(0.012)
        part.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(
                xyz=(lower_offset[0], lower_offset[1], -0.033 + lower_offset[2]),
                rpy=axis_rpy,
            ),
            material=bushing_urethane,
            name="lower_bushing",
        )
        part.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(
                xyz=(upper_offset[0], upper_offset[1], -0.033 + upper_offset[2]),
                rpy=axis_rpy,
            ),
            material=bushing_urethane,
            name="upper_bushing",
        )
        for x_off in (-0.022, 0.022):
            for y_off in (-0.020, 0.020):
                part.visual(
                    Cylinder(radius=0.0048, length=0.003),
                    origin=Origin(xyz=(x_off, y_off, -0.0085)),
                    material=stainless,
                    name=f"mount_washer_{'n' if y_off > 0.0 else 's'}_{'r' if x_off > 0.0 else 'l'}",
                )
        part.inertial = Inertial.from_geometry(
            Box((0.120, 0.080, 0.045)),
            mass=0.38,
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
        )
        return part

    def _build_hanger(part_name: str) -> object:
        part = model.part(part_name)
        part.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hanger_metal,
            name="pivot_collar",
        )
        part.visual(
            Box((0.040, 0.096, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
            material=hanger_metal,
            name="hanger_body",
        )
        part.visual(
            Box((0.016, 0.030, 0.010)),
            origin=Origin(xyz=(-0.022, 0.0, -0.023)),
            material=hanger_metal,
            name="pivot_nose",
        )
        part.visual(
            Cylinder(radius=0.0045, length=0.320),
            origin=Origin(xyz=(0.0, 0.0, -0.014), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name="axle",
        )
        for side_sign, label in ((1.0, "left"), (-1.0, "right")):
            part.visual(
                Box((0.024, 0.032, 0.020)),
                origin=Origin(xyz=(0.0, side_sign * 0.063, -0.010)),
                material=hanger_metal,
                name=f"{label}_shoulder",
            )
            part.visual(
                Cylinder(radius=0.022, length=0.004),
                origin=Origin(
                    xyz=(0.0, side_sign * 0.088, -0.014),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=stainless,
                name=f"{label}_inner_washer",
            )
            part.visual(
                Cylinder(radius=0.006, length=0.008),
                origin=Origin(
                    xyz=(0.0, side_sign * 0.149, -0.014),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=stainless,
                name=f"{label}_axle_nut",
            )
        part.inertial = Inertial.from_geometry(
            Box((0.060, 0.330, 0.050)),
            mass=0.62,
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
        )
        return part

    def _build_wheel(part_name: str) -> object:
        part = model.part(part_name)
        part.visual(wheel_mesh, material=wheel_urethane, name="wheel_shell")
        part.inertial = Inertial.from_geometry(
            Cylinder(radius=0.033, length=0.050),
            mass=0.22,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        )
        return part

    front_baseplate = _build_baseplate("front_baseplate")
    rear_baseplate = _build_baseplate("rear_baseplate")
    front_hanger = _build_hanger("front_hanger")
    rear_hanger = _build_hanger("rear_hanger")
    front_left_wheel = _build_wheel("front_left_wheel")
    front_right_wheel = _build_wheel("front_right_wheel")
    rear_left_wheel = _build_wheel("rear_left_wheel")
    rear_right_wheel = _build_wheel("rear_right_wheel")

    model.articulation(
        "deck_to_front_baseplate",
        ArticulationType.FIXED,
        parent=deck,
        child=front_baseplate,
        origin=Origin(xyz=(truck_x, 0.0, -0.012)),
    )
    model.articulation(
        "deck_to_rear_baseplate",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_baseplate,
        origin=Origin(xyz=(-truck_x, 0.0, -0.012)),
    )
    for baseplate_name, hanger_name, joint_name in (
        ("front_baseplate", "front_hanger", "front_truck_steer"),
        ("rear_baseplate", "rear_hanger", "rear_truck_steer"),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=baseplate_name,
            child=hanger_name,
            origin=Origin(xyz=(0.0, 0.0, -0.033), rpy=(0.0, kingpin_pitch, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=3.0,
                lower=-0.32,
                upper=0.32,
            ),
        )

    wheel_mounts = (
        ("front_hanger", "front_left_wheel", "front_left_spin", 0.115),
        ("front_hanger", "front_right_wheel", "front_right_spin", -0.115),
        ("rear_hanger", "rear_left_wheel", "rear_left_spin", 0.115),
        ("rear_hanger", "rear_right_wheel", "rear_right_spin", -0.115),
    )
    for hanger_name, wheel_name, joint_name, y_pos in wheel_mounts:
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=hanger_name,
            child=wheel_name,
            origin=Origin(xyz=(0.0, y_pos, -0.014)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=32.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_baseplate = object_model.get_part("front_baseplate")
    rear_baseplate = object_model.get_part("rear_baseplate")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_truck_steer = object_model.get_articulation("front_truck_steer")
    rear_truck_steer = object_model.get_articulation("rear_truck_steer")
    spin_joints = [
        object_model.get_articulation("front_left_spin"),
        object_model.get_articulation("front_right_spin"),
        object_model.get_articulation("rear_left_spin"),
        object_model.get_articulation("rear_right_spin"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(front_baseplate, deck, elem_a="gasket_pad", name="front_baseplate_sealed_to_deck")
    ctx.expect_contact(rear_baseplate, deck, elem_a="gasket_pad", name="rear_baseplate_sealed_to_deck")
    ctx.expect_contact(front_hanger, front_baseplate, name="front_hanger_supported_by_bushing_stack")
    ctx.expect_contact(rear_hanger, rear_baseplate, name="rear_hanger_supported_by_bushing_stack")

    ctx.expect_contact(
        front_left_wheel,
        front_hanger,
        elem_b="left_inner_washer",
        name="front_left_wheel_bearing_stack_contacts_inner_washer",
    )
    ctx.expect_contact(
        front_right_wheel,
        front_hanger,
        elem_b="right_inner_washer",
        name="front_right_wheel_bearing_stack_contacts_inner_washer",
    )
    ctx.expect_contact(
        rear_left_wheel,
        rear_hanger,
        elem_b="left_inner_washer",
        name="rear_left_wheel_bearing_stack_contacts_inner_washer",
    )
    ctx.expect_contact(
        rear_right_wheel,
        rear_hanger,
        elem_b="right_inner_washer",
        name="rear_right_wheel_bearing_stack_contacts_inner_washer",
    )

    ctx.check(
        "truck_steer_axes_are_longitudinal",
        front_truck_steer.axis == (1.0, 0.0, 0.0) and rear_truck_steer.axis == (1.0, 0.0, 0.0),
        details=f"front axis={front_truck_steer.axis}, rear axis={rear_truck_steer.axis}",
    )
    ctx.check(
        "wheel_spin_axes_are_axial_and_continuous",
        all(joint.axis == (0.0, 1.0, 0.0) for joint in spin_joints)
        and all(joint.motion_limits is not None for joint in spin_joints)
        and all(
            joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            for joint in spin_joints
        ),
        details=", ".join(f"{joint.name}: axis={joint.axis}, limits={joint.motion_limits}" for joint in spin_joints),
    )

    with ctx.pose({front_truck_steer: 0.24}):
        left_z = ctx.part_world_position(front_left_wheel)[2]
        right_z = ctx.part_world_position(front_right_wheel)[2]
        ctx.check(
            "front_truck_steer_tilts_axle_line",
            abs(left_z - right_z) > 0.010,
            details=f"left_z={left_z:.4f}, right_z={right_z:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
