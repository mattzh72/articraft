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
)


PALM_WIDTH = 0.082
PALM_LENGTH = 0.084
PALM_THICKNESS = 0.024
PALM_FRONT_Y = PALM_LENGTH * 0.5
LINK_BODY_OFFSET = 0.006
LINK_TIP_MARGIN = 0.006
HINGE_PAD_DEPTH = 0.006
BASE_FRONT_SETBACK = 0.004

FINGER_DIGITS = (
    {
        "name": "index",
        "base_x": -0.024,
        "yaw": math.radians(-4.0),
        "base_width": 0.014,
        "base_depth": 0.014,
        "base_height": 0.030,
        "thickness": 0.016,
        "lengths": (0.034, 0.024, 0.018),
    },
    {
        "name": "middle",
        "base_x": -0.008,
        "yaw": 0.0,
        "base_width": 0.015,
        "base_depth": 0.015,
        "base_height": 0.032,
        "thickness": 0.017,
        "lengths": (0.038, 0.028, 0.020),
    },
    {
        "name": "ring",
        "base_x": 0.009,
        "yaw": math.radians(3.0),
        "base_width": 0.0145,
        "base_depth": 0.015,
        "base_height": 0.031,
        "thickness": 0.0165,
        "lengths": (0.035, 0.026, 0.019),
    },
    {
        "name": "little",
        "base_x": 0.025,
        "yaw": math.radians(7.0),
        "base_width": 0.0125,
        "base_depth": 0.014,
        "base_height": 0.029,
        "thickness": 0.015,
        "lengths": (0.028, 0.020, 0.016),
    },
)

THUMB_DIGIT = {
    "name": "thumb",
    "base_xy": (-0.034, -0.006),
    "yaw": math.radians(-52.0),
    "base_width": 0.016,
    "base_depth": 0.018,
    "base_height": 0.027,
    "thickness": 0.017,
    "lengths": (0.028, 0.021),
}

FINGER_JOINT_LIMITS = (
    ("mcp", -0.60, 0.0),
    ("pip", -1.00, 0.0),
    ("dip", -0.80, 0.0),
)

THUMB_JOINT_LIMITS = (
    ("carpometacarpal", -0.45, 0.0),
    ("interphalangeal", -0.70, 0.0),
)


def _add_base_mount(
    part,
    *,
    width: float,
    depth: float,
    height: float,
    joint_thickness: float,
    shell_material,
    joint_material,
) -> None:
    part.visual(
        Box((width, depth, height)),
        origin=Origin(
            xyz=(
                0.0,
                -(BASE_FRONT_SETBACK + depth * 0.5),
                -height * 0.5,
            )
        ),
        material=shell_material,
        name="base_block",
    )
    part.visual(
        Box((width * 0.62, HINGE_PAD_DEPTH, joint_thickness * 0.40)),
        origin=Origin(
            xyz=(
                0.0,
                -HINGE_PAD_DEPTH * 0.5,
                -joint_thickness * 0.20,
            )
        ),
        material=joint_material,
        name="hinge_pad",
    )


def _add_phalanx(
    part,
    *,
    width: float,
    length: float,
    thickness: float,
    shell_material,
    joint_material,
    add_distal_pin: bool = True,
) -> None:
    body_start = min(LINK_BODY_OFFSET, length * 0.25)
    body_end_margin = LINK_TIP_MARGIN if add_distal_pin else max(LINK_TIP_MARGIN * 0.75, 0.003)
    body_length = max(length - body_start - body_end_margin, 0.004)
    hinge_height = thickness * 0.42
    body_height = thickness * 0.70
    part.visual(
        Box((width * 0.56, HINGE_PAD_DEPTH, hinge_height)),
        origin=Origin(xyz=(0.0, HINGE_PAD_DEPTH * 0.5, hinge_height * 0.5)),
        material=joint_material,
        name="proximal_hinge_block",
    )
    part.visual(
        Box((width * 0.84, body_length, body_height)),
        origin=Origin(
            xyz=(
                0.0,
                body_start + body_length * 0.5,
                body_height * 0.5,
            )
        ),
        material=shell_material,
        name="body",
    )
    if add_distal_pin:
        part.visual(
            Box((width * 0.52, HINGE_PAD_DEPTH, hinge_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    length - HINGE_PAD_DEPTH * 0.5,
                    hinge_height * 0.5,
                )
            ),
            material=joint_material,
            name="distal_hinge_block",
        )


def _visual_center_from_aabb(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def _base_mount_origin(base_height: float) -> float:
    return PALM_THICKNESS + base_height


def _body_origin_y(length: float, has_distal_hinge: bool) -> float:
    body_start = min(LINK_BODY_OFFSET, length * 0.25)
    body_end_margin = LINK_TIP_MARGIN if has_distal_hinge else max(LINK_TIP_MARGIN * 0.75, 0.003)
    body_length = max(length - body_start - body_end_margin, 0.004)
    return body_start + body_length * 0.5


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_hand_palm")

    palm_shell = model.material("palm_shell", rgba=(0.23, 0.24, 0.27, 1.0))
    wrist_shell = model.material("wrist_shell", rgba=(0.18, 0.19, 0.21, 1.0))
    phalanx_shell = model.material("phalanx_shell", rgba=(0.80, 0.82, 0.84, 1.0))
    joint_shell = model.material("joint_shell", rgba=(0.55, 0.58, 0.61, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_WIDTH, PALM_LENGTH, PALM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PALM_THICKNESS * 0.5)),
        material=palm_shell,
        name="palm_block",
    )
    palm.visual(
        Box((0.050, 0.026, 0.020)),
        origin=Origin(
            xyz=(
                0.0,
                -PALM_LENGTH * 0.5 - 0.013,
                0.010,
            )
        ),
        material=wrist_shell,
        name="wrist_stub",
    )
    palm.inertial = Inertial.from_geometry(
        Box((PALM_WIDTH, PALM_LENGTH + 0.026, PALM_THICKNESS)),
        mass=0.65,
        origin=Origin(xyz=(0.0, -0.006, PALM_THICKNESS * 0.5)),
    )

    for digit in FINGER_DIGITS:
        name = digit["name"]
        base_width = digit["base_width"]
        base_depth = digit["base_depth"]
        thickness = digit["thickness"]
        lengths = digit["lengths"]
        base_height = digit["base_height"]
        base_z = _base_mount_origin(base_height)

        base = model.part(f"{name}_base")
        _add_base_mount(
            base,
            width=base_width,
            depth=base_depth,
            height=base_height,
            joint_thickness=thickness,
            shell_material=phalanx_shell,
            joint_material=joint_shell,
        )
        base.inertial = Inertial.from_geometry(
            Box((base_width, base_depth, base_height)),
            mass=0.035,
            origin=Origin(
                xyz=(
                    0.0,
                    -(BASE_FRONT_SETBACK + base_depth * 0.5),
                    -base_height * 0.5,
                )
            ),
        )
        model.articulation(
            f"palm_to_{name}_base",
            ArticulationType.FIXED,
            parent=palm,
            child=base,
            origin=Origin(
                xyz=(digit["base_x"], PALM_FRONT_Y + BASE_FRONT_SETBACK, base_z),
                rpy=(0.0, 0.0, digit["yaw"]),
            ),
        )

        parent_part = base
        parent_joint_y = 0.0
        current_width = base_width * 0.96
        masses = (0.028, 0.022, 0.018)
        segment_names = ("proximal", "middle", "distal")

        for segment_index, (segment_name, segment_length, segment_mass) in enumerate(
            zip(segment_names, lengths, masses)
        ):
            segment = model.part(f"{name}_{segment_name}")
            _add_phalanx(
                segment,
                width=current_width,
                length=segment_length,
                thickness=thickness,
                shell_material=phalanx_shell,
                joint_material=joint_shell,
                add_distal_pin=segment_name != "distal",
            )
            segment.inertial = Inertial.from_geometry(
                Box(
                    (
                        current_width * 0.84,
                        max(segment_length - LINK_BODY_OFFSET - LINK_TIP_MARGIN, 0.004),
                        thickness * 0.86,
                    )
                ),
                mass=segment_mass,
                origin=Origin(
                    xyz=(
                        0.0,
                        _body_origin_y(segment_length, segment_name != "distal"),
                        0.0,
                    )
                ),
            )

            lower, upper = FINGER_JOINT_LIMITS[segment_index][1:]
            model.articulation(
                f"{name}_{segment_name}_hinge",
                ArticulationType.REVOLUTE,
                parent=parent_part,
                child=segment,
                origin=Origin(xyz=(0.0, parent_joint_y, 0.0)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(
                    effort=2.5,
                    velocity=4.0,
                    lower=lower,
                    upper=upper,
                ),
            )
            parent_part = segment
            parent_joint_y = segment_length
            current_width *= 0.90

    thumb = THUMB_DIGIT
    thumb_base = model.part("thumb_base")
    _add_base_mount(
        thumb_base,
        width=thumb["base_width"],
        depth=thumb["base_depth"],
        height=thumb["base_height"],
        joint_thickness=thumb["thickness"],
        shell_material=phalanx_shell,
        joint_material=joint_shell,
    )
    thumb_base.inertial = Inertial.from_geometry(
        Box((thumb["base_width"], thumb["base_depth"], thumb["base_height"])),
        mass=0.04,
        origin=Origin(
            xyz=(
                0.0,
                -(BASE_FRONT_SETBACK + thumb["base_depth"] * 0.5),
                -thumb["base_height"] * 0.5,
            )
        ),
    )
    model.articulation(
        "palm_to_thumb_base",
        ArticulationType.FIXED,
        parent=palm,
        child=thumb_base,
        origin=Origin(
            xyz=(
                thumb["base_xy"][0],
                thumb["base_xy"][1],
                _base_mount_origin(thumb["base_height"]),
            ),
            rpy=(0.0, 0.0, thumb["yaw"]),
        ),
    )

    thumb_widths = (thumb["base_width"] * 0.92, thumb["base_width"] * 0.82)
    thumb_masses = (0.026, 0.020)
    thumb_parent = thumb_base
    thumb_parent_joint_y = 0.0
    for segment_index, segment_name in enumerate(("proximal", "distal")):
        length = thumb["lengths"][segment_index]
        width = thumb_widths[segment_index]
        segment = model.part(f"thumb_{segment_name}")
        _add_phalanx(
            segment,
            width=width,
            length=length,
            thickness=thumb["thickness"],
            shell_material=phalanx_shell,
            joint_material=joint_shell,
            add_distal_pin=segment_name != "distal",
        )
        segment.inertial = Inertial.from_geometry(
            Box(
                (
                    width * 0.84,
                    max(length - LINK_BODY_OFFSET - LINK_TIP_MARGIN, 0.004),
                    thumb["thickness"] * 0.86,
                )
            ),
            mass=thumb_masses[segment_index],
            origin=Origin(
                xyz=(
                    0.0,
                    _body_origin_y(length, segment_name != "distal"),
                    0.0,
                )
            ),
        )

        lower, upper = THUMB_JOINT_LIMITS[segment_index][1:]
        model.articulation(
            f"thumb_{segment_name}_hinge",
            ArticulationType.REVOLUTE,
            parent=thumb_parent,
            child=segment,
            origin=Origin(xyz=(0.0, thumb_parent_joint_y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=4.0,
                lower=lower,
                upper=upper,
            ),
        )
        thumb_parent = segment
        thumb_parent_joint_y = length

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    hinge_pairs: list[tuple[object, object, str]] = []

    for digit in FINGER_DIGITS:
        name = digit["name"]
        base = object_model.get_part(f"{name}_base")
        proximal = object_model.get_part(f"{name}_proximal")
        middle = object_model.get_part(f"{name}_middle")
        distal = object_model.get_part(f"{name}_distal")

        ctx.expect_gap(
            base,
            palm,
            axis="z",
            max_gap=1e-5,
            max_penetration=0.0,
            name=f"{name}_base_on_palm",
        )
        ctx.expect_overlap(
            base,
            palm,
            axes="xy",
            min_overlap=0.006,
            name=f"{name}_base_footprint_on_palm",
        )
        ctx.expect_contact(base, palm, name=f"{name}_base_contacts_palm")

        hinge_pairs.extend(
            (
                (base, proximal, f"{name}_proximal_hinge"),
                (proximal, middle, f"{name}_middle_hinge"),
                (middle, distal, f"{name}_distal_hinge"),
            )
        )

        tip_aabb = ctx.part_element_world_aabb(distal, elem="body")
        base_pose = ctx.part_world_position(base)
        assert tip_aabb is not None and base_pose is not None
        tip_pose = _visual_center_from_aabb(tip_aabb)
        ctx.check(
            f"{name}_finger_reaches_forward",
            tip_pose[1] > base_pose[1] + 0.035,
            details=f"Expected {name} fingertip ahead of base, got {tip_pose} vs {base_pose}.",
        )

    thumb_base = object_model.get_part("thumb_base")
    thumb_proximal = object_model.get_part("thumb_proximal")
    thumb_distal = object_model.get_part("thumb_distal")

    ctx.expect_gap(
        thumb_base,
        palm,
        axis="z",
        max_gap=1e-5,
        max_penetration=0.0,
        name="thumb_base_on_palm",
    )
    ctx.expect_overlap(
        thumb_base,
        palm,
        axes="xy",
        min_overlap=0.004,
        name="thumb_base_footprint_on_palm",
    )
    ctx.expect_contact(thumb_base, palm, name="thumb_base_contacts_palm")
    hinge_pairs.extend(
        (
            (thumb_base, thumb_proximal, "thumb_proximal_hinge"),
            (thumb_proximal, thumb_distal, "thumb_distal_hinge"),
        )
    )

    thumb_base_position = ctx.part_world_position(thumb_base)
    thumb_tip_aabb = ctx.part_element_world_aabb(thumb_distal, elem="body")
    index_base_position = ctx.part_world_position(object_model.get_part("index_base"))
    assert (
        thumb_base_position is not None
        and thumb_tip_aabb is not None
        and index_base_position is not None
    )
    thumb_tip_position = _visual_center_from_aabb(thumb_tip_aabb)
    ctx.check(
        "thumb_opposes_fingers",
        thumb_base_position[0] < index_base_position[0]
        and thumb_tip_position[0] > thumb_base_position[0] + 0.010,
        details=(
            "Thumb should mount to the side of the palm and point inward. "
            f"base={thumb_base_position}, tip={thumb_tip_position}, index_base={index_base_position}"
        ),
    )

    ctx.fail_if_parts_overlap_in_current_pose()

    all_joint_names = [
        f"{digit['name']}_proximal_hinge" for digit in FINGER_DIGITS
    ] + [
        f"{digit['name']}_middle_hinge" for digit in FINGER_DIGITS
    ] + [
        f"{digit['name']}_distal_hinge" for digit in FINGER_DIGITS
    ] + [
        "thumb_proximal_hinge",
        "thumb_distal_hinge",
    ]

    for joint_name in all_joint_names:
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        assert limits is not None and limits.lower is not None and limits.upper is not None
        ctx.check(
            f"{joint_name}_axis_is_widthwise",
            abs(joint.axis[0]) > 0.99 and abs(joint.axis[1]) < 1e-9 and abs(joint.axis[2]) < 1e-9,
            details=f"Expected hinge axis across link width, got {joint.axis}.",
        )
        ctx.check(
            f"{joint_name}_has_bending_range",
            limits.lower < limits.upper,
            details=f"Expected non-zero motion range, got lower={limits.lower}, upper={limits.upper}.",
        )

        child = object_model.get_part(joint.child)
        tracked_elem = "body"

        with ctx.pose({joint: limits.upper}):
            upper_aabb = ctx.part_element_world_aabb(child, elem=tracked_elem)
            assert upper_aabb is not None
            upper_center = _visual_center_from_aabb(upper_aabb)
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint_name}_upper_no_floating")

        with ctx.pose({joint: limits.lower}):
            lower_aabb = ctx.part_element_world_aabb(child, elem=tracked_elem)
            assert lower_aabb is not None
            lower_center = _visual_center_from_aabb(lower_aabb)
            ctx.check(
                f"{joint_name}_flexes_downward",
                lower_center[2] < upper_center[2] - 0.002,
                details=(
                    f"Expected flexed pose to lower the child segment, "
                    f"got lower={lower_center}, upper={upper_center}."
                ),
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint_name}_lower_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
