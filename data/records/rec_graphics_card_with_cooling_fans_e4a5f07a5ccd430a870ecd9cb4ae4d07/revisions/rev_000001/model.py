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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_gaming_graphics_card")

    card_black = model.material("card_black", rgba=(0.08, 0.09, 0.10, 1.0))
    shroud_black = model.material("shroud_black", rgba=(0.11, 0.12, 0.13, 1.0))
    heatsink_gray = model.material("heatsink_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.65, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.20, 0.22, 1.0))
    accent = model.material("accent", rgba=(0.42, 0.46, 0.50, 1.0))

    card_length = 0.318
    card_height = 0.110
    card_thickness = 0.040
    rotor_radius = 0.036
    fan_centers_x = (0.068, 0.159, 0.250)

    def circle_profile(
        radius: float,
        *,
        segments: int = 36,
    ) -> list[tuple[float, float]]:
        return [
            (
                radius * math.cos(2.0 * math.pi * idx / segments),
                radius * math.sin(2.0 * math.pi * idx / segments),
            )
            for idx in range(segments)
        ]

    def add_fan_rotor(part_name: str) -> None:
        rotor = model.part(part_name)
        rotor.visual(
            Cylinder(radius=0.0135, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=dark_metal,
            name="hub_shell",
        )
        rotor.visual(
            Cylinder(radius=0.0065, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=steel,
            name="hub_cap",
        )
        for blade_idx in range(7):
            angle = 2.0 * math.pi * blade_idx / 7.0
            rotor.visual(
                Box((0.034, 0.011, 0.003)),
                origin=Origin(
                    xyz=(0.018 * math.cos(angle), 0.018 * math.sin(angle), 0.005),
                    rpy=(0.0, 0.28, angle),
                ),
                material=dark_metal,
                name=f"blade_{blade_idx}",
            )
        rotor.inertial = Inertial.from_geometry(
            Cylinder(radius=rotor_radius, length=0.012),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
        )

    body = model.part("card_body")
    body.inertial = Inertial.from_geometry(
        Box((card_length, card_height, card_thickness)),
        mass=1.7,
        origin=Origin(xyz=(card_length * 0.5, 0.0, card_thickness * 0.5)),
    )
    body.visual(
        Box((card_length, card_height, 0.002)),
        origin=Origin(xyz=(card_length * 0.5, 0.0, 0.001)),
        material=card_black,
        name="pcb_board",
    )
    body.visual(
        Box((0.306, 0.104, 0.020)),
        origin=Origin(xyz=(0.160, 0.0, 0.012)),
        material=heatsink_gray,
        name="heatsink_core",
    )

    body.visual(
        Box((0.304, 0.008, 0.020)),
        origin=Origin(xyz=(0.160, 0.051, 0.029)),
        material=shroud_black,
        name="upper_rail_right",
    )
    body.visual(
        Box((0.304, 0.008, 0.020)),
        origin=Origin(xyz=(0.160, -0.051, 0.029)),
        material=shroud_black,
        name="upper_rail_left",
    )
    body.visual(
        Box((0.010, 0.090, 0.018)),
        origin=Origin(xyz=(0.004, 0.0, 0.029)),
        material=shroud_black,
        name="bracket_shroud_block",
    )
    body.visual(
        Box((0.016, 0.094, 0.020)),
        origin=Origin(xyz=(0.015, 0.0, 0.029)),
        material=shroud_black,
        name="bracket_end_frame",
    )
    body.visual(
        Box((0.016, 0.094, 0.020)),
        origin=Origin(xyz=(0.1135, 0.0, 0.029)),
        material=shroud_black,
        name="fan_bridge_left",
    )
    body.visual(
        Box((0.016, 0.094, 0.020)),
        origin=Origin(xyz=(0.2045, 0.0, 0.029)),
        material=shroud_black,
        name="fan_bridge_right",
    )
    body.visual(
        Box((0.012, 0.108, 0.020)),
        origin=Origin(xyz=(0.312, 0.0, 0.029)),
        material=shroud_black,
        name="front_nose",
    )
    body.visual(
        Box((0.170, 0.004, 0.004)),
        origin=Origin(xyz=(0.188, 0.047, 0.038)),
        material=accent,
        name="top_edge_accent",
    )
    body.visual(
        Box((0.002, 0.114, 0.042)),
        origin=Origin(xyz=(-0.001, 0.0, 0.021)),
        material=steel,
        name="io_bracket_plate",
    )
    body.visual(
        Box((0.050, 0.010, 0.004)),
        origin=Origin(xyz=(0.032, -0.050, 0.002)),
        material=steel,
        name="pcie_fingers",
    )
    body.visual(
        Box((0.006, 0.016, 0.014)),
        origin=Origin(xyz=(-0.001, -0.038, -0.006)),
        material=steel,
        name="brace_mount_lug",
    )
    for bearing_idx, fan_x in enumerate(fan_centers_x, start=1):
        body.visual(
            Cylinder(radius=0.0035, length=0.010),
            origin=Origin(xyz=(fan_x, 0.0, 0.023)),
            material=steel,
            name=f"fan_bearing_{bearing_idx}",
        )
    for bezel_idx, fan_x in enumerate(fan_centers_x, start=1):
        bezel_mesh = mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                circle_profile(0.051),
                [circle_profile(0.043)],
                height=0.006,
                center=True,
            ),
            f"fan_bezel_{bezel_idx}",
        )
        body.visual(
            bezel_mesh,
            origin=Origin(xyz=(fan_x, 0.0, 0.036)),
            material=shroud_black,
            name=f"fan_bezel_{bezel_idx}",
        )

    add_fan_rotor("fan_left")
    add_fan_rotor("fan_center")
    add_fan_rotor("fan_right")

    brace = model.part("support_brace")
    brace.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_metal,
        name="brace_hinge_barrel",
    )
    brace.visual(
        Box((0.018, 0.014, 0.052)),
        origin=Origin(xyz=(-0.012, 0.0, -0.026)),
        material=dark_metal,
        name="brace_arm",
    )
    brace.visual(
        Box((0.018, 0.016, 0.008)),
        origin=Origin(xyz=(-0.004, 0.0, -0.052)),
        material=steel,
        name="brace_foot",
    )
    brace.inertial = Inertial.from_geometry(
        Box((0.024, 0.018, 0.070)),
        mass=0.05,
        origin=Origin(xyz=(-0.006, 0.0, -0.030)),
    )

    model.articulation(
        "body_to_left_fan",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="fan_left",
        origin=Origin(xyz=(fan_centers_x[0], 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=60.0),
    )
    model.articulation(
        "body_to_center_fan",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="fan_center",
        origin=Origin(xyz=(fan_centers_x[1], 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=60.0),
    )
    model.articulation(
        "body_to_right_fan",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="fan_right",
        origin=Origin(xyz=(fan_centers_x[2], 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=60.0),
    )
    model.articulation(
        "body_to_support_brace",
        ArticulationType.REVOLUTE,
        parent=body,
        child=brace,
        origin=Origin(xyz=(-0.0075, -0.038, -0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(82.0),
        ),
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
    body = object_model.get_part("card_body")
    fans = [
        ("fan_left", "body_to_left_fan"),
        ("fan_center", "body_to_center_fan"),
        ("fan_right", "body_to_right_fan"),
    ]
    brace = object_model.get_part("support_brace")
    brace_joint = object_model.get_articulation("body_to_support_brace")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[idx] + maxs[idx]) * 0.5 for idx in range(3))

    for fan_name, joint_name in fans:
        fan = object_model.get_part(fan_name)
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{fan_name} uses continuous axial spin",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_gap(
            fan,
            body,
            axis="z",
            min_gap=0.004,
            max_gap=0.012,
            negative_elem="heatsink_core",
            name=f"{fan_name} clears the heatsink block",
        )
        ctx.expect_overlap(
            fan,
            body,
            axes="xy",
            min_overlap=0.068,
            name=f"{fan_name} stays centered within the card face",
        )

    brace_limits = brace_joint.motion_limits
    ctx.check(
        "support brace hinge opens from the bracket edge",
        brace_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(brace_joint.axis) == (0.0, -1.0, 0.0)
        and brace_limits is not None
        and brace_limits.lower == 0.0
        and brace_limits.upper is not None
        and brace_limits.upper > math.radians(70.0),
        details=f"type={brace_joint.articulation_type}, axis={brace_joint.axis}, limits={brace_limits}",
    )

    closed_center = aabb_center(ctx.part_element_world_aabb(brace, elem="brace_foot"))
    with ctx.pose({brace_joint: math.radians(78.0)}):
        open_center = aabb_center(ctx.part_element_world_aabb(brace, elem="brace_foot"))
        ctx.expect_gap(
            body,
            brace,
            axis="z",
            min_gap=0.001,
            max_gap=0.020,
            positive_elem="pcb_board",
            name="opened brace stays below the card body",
        )

    ctx.check(
        "support brace swings outward from the bracket",
        closed_center is not None
        and open_center is not None
        and open_center[0] > closed_center[0] + 0.040
        and open_center[1] > closed_center[1] - 0.010,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
