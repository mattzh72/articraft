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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PALM_THICKNESS = 0.018
PALM_WIDTH = 0.150
PALM_HEIGHT = 0.100
PALM_FRONT_X = 0.032
FINGER_HINGE_RADIUS = 0.0045
FINGER_BOSS_LENGTH = 0.008
FINGER_EAR_LENGTH = 0.005
THUMB_HINGE_RADIUS = 0.0045
THUMB_BOSS_LENGTH = 0.010
THUMB_EAR_LENGTH = 0.006

FINGER_CHAINS = (
    {
        "name": "little",
        "root_y": -0.045,
        "root_z": 0.022,
        "width": 0.015,
        "thickness": 0.011,
        "lengths": (0.038, 0.025, 0.019),
        "limits": (0.95, 1.05, 0.90),
    },
    {
        "name": "ring",
        "root_y": -0.016,
        "root_z": 0.029,
        "width": 0.016,
        "thickness": 0.0115,
        "lengths": (0.045, 0.029, 0.021),
        "limits": (1.00, 1.10, 0.92),
    },
    {
        "name": "middle",
        "root_y": 0.013,
        "root_z": 0.034,
        "width": 0.017,
        "thickness": 0.012,
        "lengths": (0.050, 0.032, 0.023),
        "limits": (1.02, 1.12, 0.95),
    },
    {
        "name": "index",
        "root_y": 0.041,
        "root_z": 0.031,
        "width": 0.016,
        "thickness": 0.0115,
        "lengths": (0.044, 0.029, 0.021),
        "limits": (0.98, 1.08, 0.90),
    },
)

THUMB_CHAIN = {
    "name": "thumb",
    "root_y": 0.076,
    "root_z": -0.012,
    "width": 0.018,
    "thickness": 0.016,
    "lengths": (0.036, 0.026, 0.020),
    "limits": (0.95, 1.05, 0.88),
}

LINK_SEGMENT_NAMES = ("proximal", "middle", "distal")


def _finger_joint_name(chain_name: str, parent_label: str, child_label: str) -> str:
    return f"{chain_name}_{parent_label}_to_{child_label}"


def _link_part_name(chain_name: str, segment_name: str) -> str:
    return f"{chain_name}_{segment_name}"


def _x_rounded_section(x: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    fillet = min(radius, 0.48 * width, 0.48 * height)
    return [
        (y, z, x)
        for y, z in rounded_rect_profile(
            width,
            height,
            fillet,
            corner_segments=5,
        )
    ]


def _palm_plate_mesh():
    geom = LoftGeometry(
        [
        _x_rounded_section(0.0, PALM_WIDTH, PALM_HEIGHT, 0.012),
        _x_rounded_section(0.5 * PALM_THICKNESS, PALM_WIDTH, PALM_HEIGHT, 0.012),
        _x_rounded_section(PALM_THICKNESS, PALM_WIDTH, PALM_HEIGHT, 0.012),
        ],
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0).rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, "palm_plate_shell")


def _finger_shell_mesh(name: str, *, length: float, width: float, thickness: float, distal: bool):
    beam_start = FINGER_HINGE_RADIUS
    beam_end = length - (FINGER_HINGE_RADIUS if not distal else 0.0)
    beam_length = beam_end - beam_start
    corner = min(width, thickness) * 0.18
    geom = LoftGeometry(
        [
            _x_rounded_section(beam_start, width * 0.50, thickness * 0.78, corner),
            _x_rounded_section(beam_start + 0.50 * beam_length, width * 0.68, thickness * 0.96, corner),
            _x_rounded_section(
                beam_end,
                width * (0.44 if distal else 0.56),
                thickness * (0.66 if distal else 0.80),
                corner * 0.9,
            ),
        ],
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0).rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _thumb_shell_mesh(name: str, *, length: float, width: float, thickness: float, distal: bool):
    beam_start = THUMB_HINGE_RADIUS
    beam_end = length - (THUMB_HINGE_RADIUS if not distal else 0.0)
    beam_length = beam_end - beam_start
    corner = min(width, thickness) * 0.20
    geom = LoftGeometry(
        [
            _x_rounded_section(beam_start, width * 0.56, thickness * 0.64, corner),
            _x_rounded_section(beam_start + 0.48 * beam_length, width * 0.82, thickness * 0.82, corner),
            _x_rounded_section(
                beam_end,
                width * (0.48 if distal else 0.62),
                thickness * (0.58 if distal else 0.70),
                corner * 0.9,
            ),
        ],
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0).rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _finger_link_part(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    width: float,
    thickness: float,
    structure_material,
    pad_material,
    distal: bool,
):
    part = model.part(name)

    ear_offset = 0.5 * (FINGER_BOSS_LENGTH + FINGER_EAR_LENGTH)
    y_cylinder = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    beam_start = FINGER_HINGE_RADIUS
    beam_end = length - (FINGER_HINGE_RADIUS if not distal else 0.0)
    beam_length = beam_end - beam_start
    if beam_length <= 0.0:
        raise ValueError(f"Invalid finger beam length for {name}")

    part.visual(
        Cylinder(radius=FINGER_HINGE_RADIUS, length=FINGER_EAR_LENGTH),
        origin=Origin(xyz=(0.0, ear_offset, 0.0), rpy=y_cylinder.rpy),
        material=structure_material,
        name="proximal_ear_upper",
    )
    part.visual(
        Cylinder(radius=FINGER_HINGE_RADIUS, length=FINGER_EAR_LENGTH),
        origin=Origin(xyz=(0.0, -ear_offset, 0.0), rpy=y_cylinder.rpy),
        material=structure_material,
        name="proximal_ear_lower",
    )
    for side in (-1.0, 1.0):
        part.visual(
            Box((0.012, FINGER_EAR_LENGTH, thickness)),
            origin=Origin(
                xyz=(
                    0.006,
                    side * ear_offset,
                    0.0,
                )
            ),
            material=structure_material,
            name=f"proximal_web_{'upper' if side > 0.0 else 'lower'}",
        )

    part.visual(
        _finger_shell_mesh(
            f"{name}_shell",
            length=length,
            width=width,
            thickness=thickness,
            distal=distal,
        ),
        material=structure_material,
        name="beam_shell",
    )

    if distal:
        tip_length = min(length * 0.26, 0.010)
        part.visual(
            Box((tip_length, width * 0.82, thickness * 0.52)),
            origin=Origin(
                xyz=(
                    length - 0.5 * tip_length,
                    0.0,
                    -0.38 * thickness,
                )
            ),
            material=pad_material,
            name="pad",
        )
    else:
        part.visual(
            Cylinder(radius=FINGER_HINGE_RADIUS, length=FINGER_BOSS_LENGTH),
            origin=Origin(xyz=(length, 0.0, 0.0), rpy=y_cylinder.rpy),
            material=structure_material,
            name="distal_boss",
        )

    part.inertial = Inertial.from_geometry(
        Box((length + 2.0 * FINGER_HINGE_RADIUS, width, thickness * 1.2)),
        mass=max(0.035, 1.8 * length),
        origin=Origin(xyz=(0.5 * length, 0.0, 0.0)),
    )
    return part


def _thumb_link_part(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    width: float,
    thickness: float,
    structure_material,
    pad_material,
    distal: bool,
):
    part = model.part(name)

    ear_offset = 0.5 * (THUMB_BOSS_LENGTH + THUMB_EAR_LENGTH)
    beam_start = THUMB_HINGE_RADIUS
    beam_end = length - (THUMB_HINGE_RADIUS if not distal else 0.0)
    beam_length = beam_end - beam_start
    if beam_length <= 0.0:
        raise ValueError(f"Invalid thumb beam length for {name}")

    part.visual(
        Cylinder(radius=THUMB_HINGE_RADIUS, length=THUMB_EAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, ear_offset)),
        material=structure_material,
        name="proximal_ear_upper",
    )
    part.visual(
        Cylinder(radius=THUMB_HINGE_RADIUS, length=THUMB_EAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -ear_offset)),
        material=structure_material,
        name="proximal_ear_lower",
    )
    for side in (-1.0, 1.0):
        part.visual(
            Box((0.012, width * 0.56, THUMB_EAR_LENGTH)),
            origin=Origin(
                xyz=(
                    0.006,
                    0.0,
                    side * ear_offset,
                )
            ),
            material=structure_material,
            name=f"proximal_web_{'upper' if side > 0.0 else 'lower'}",
        )

    part.visual(
        _thumb_shell_mesh(
            f"{name}_shell",
            length=length,
            width=width,
            thickness=thickness,
            distal=distal,
        ),
        material=structure_material,
        name="beam_shell",
    )

    if distal:
        tip_length = min(length * 0.30, 0.011)
        part.visual(
            Box((tip_length, width * 0.42, thickness * 0.78)),
            origin=Origin(
                xyz=(
                    length - 0.5 * tip_length,
                    -0.20 * width,
                    0.0,
                )
            ),
            material=pad_material,
            name="pad",
        )
    else:
        part.visual(
            Cylinder(radius=THUMB_HINGE_RADIUS, length=THUMB_BOSS_LENGTH),
            origin=Origin(xyz=(length, 0.0, 0.0)),
            material=structure_material,
            name="distal_boss",
        )

    part.inertial = Inertial.from_geometry(
        Box((length + 2.0 * THUMB_HINGE_RADIUS, width, thickness)),
        mass=max(0.03, 1.7 * length),
        origin=Origin(xyz=(0.5 * length, 0.0, 0.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anthropomorphic_gripper_palm")

    palm_dark = model.material("palm_dark", rgba=(0.19, 0.20, 0.23, 1.0))
    rail_dark = model.material("rail_dark", rgba=(0.25, 0.27, 0.30, 1.0))
    link_silver = model.material("link_silver", rgba=(0.77, 0.79, 0.82, 1.0))
    knuckle_steel = model.material("knuckle_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    pad_black = model.material("pad_black", rgba=(0.10, 0.11, 0.12, 1.0))

    palm = model.part("palm")
    palm.visual(
        _palm_plate_mesh(),
        material=palm_dark,
        name="palm_plate",
    )
    palm.visual(
        Box((0.026, 0.082, 0.040)),
        origin=Origin(xyz=(0.018, 0.0, -0.016)),
        material=rail_dark,
        name="lower_stiffener",
    )

    y_cylinder_rpy = (math.pi / 2.0, 0.0, 0.0)
    for chain in FINGER_CHAINS:
        palm.visual(
            Box((0.014, 0.008, 0.004)),
            origin=Origin(
                xyz=(
                    0.023,
                    chain["root_y"],
                    chain["root_z"],
                )
            ),
            material=rail_dark,
            name=f"{chain['name']}_root_bracket",
        )
        palm.visual(
            Cylinder(radius=FINGER_HINGE_RADIUS, length=FINGER_BOSS_LENGTH),
            origin=Origin(
                xyz=(
                    PALM_FRONT_X,
                    chain["root_y"],
                    chain["root_z"],
                ),
                rpy=y_cylinder_rpy,
            ),
            material=knuckle_steel,
            name=f"{chain['name']}_root_boss",
        )

    palm.visual(
        Box((0.014, 0.012, 0.010)),
        origin=Origin(
            xyz=(
                0.019,
                THUMB_CHAIN["root_y"],
                THUMB_CHAIN["root_z"],
            )
        ),
        material=rail_dark,
        name="thumb_root_bracket",
    )
    palm.visual(
        Cylinder(radius=THUMB_HINGE_RADIUS, length=THUMB_BOSS_LENGTH),
        origin=Origin(
            xyz=(
                0.026,
                THUMB_CHAIN["root_y"],
                THUMB_CHAIN["root_z"],
            )
        ),
        material=knuckle_steel,
        name="thumb_root_boss",
    )
    palm.inertial = Inertial.from_geometry(
        Box((0.040, PALM_WIDTH, PALM_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    for chain in FINGER_CHAINS:
        segment_parts = []
        for segment_name, length in zip(LINK_SEGMENT_NAMES, chain["lengths"]):
            part = _finger_link_part(
                model,
                name=_link_part_name(chain["name"], segment_name),
                length=length,
                width=chain["width"],
                thickness=chain["thickness"],
                structure_material=link_silver,
                pad_material=pad_black,
                distal=segment_name == "distal",
            )
            segment_parts.append(part)

        root_joint = model.articulation(
            _finger_joint_name(chain["name"], "palm", "proximal"),
            ArticulationType.REVOLUTE,
            parent=palm,
            child=segment_parts[0],
            origin=Origin(xyz=(PALM_FRONT_X, chain["root_y"], chain["root_z"])),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=2.5,
                lower=0.0,
                upper=chain["limits"][0],
            ),
        )
        _ = root_joint

        for index in range(2):
            parent_part = segment_parts[index]
            child_part = segment_parts[index + 1]
            parent_length = chain["lengths"][index]
            joint_name = _finger_joint_name(
                chain["name"],
                LINK_SEGMENT_NAMES[index],
                LINK_SEGMENT_NAMES[index + 1],
            )
            model.articulation(
                joint_name,
                ArticulationType.REVOLUTE,
                parent=parent_part,
                child=child_part,
                origin=Origin(xyz=(parent_length, 0.0, 0.0)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=8.0,
                    velocity=3.0,
                    lower=0.0,
                    upper=chain["limits"][index + 1],
                ),
            )

    thumb_parts = []
    for segment_name, length in zip(LINK_SEGMENT_NAMES, THUMB_CHAIN["lengths"]):
        thumb_parts.append(
            _thumb_link_part(
                model,
                name=_link_part_name(THUMB_CHAIN["name"], segment_name),
                length=length,
                width=THUMB_CHAIN["width"],
                thickness=THUMB_CHAIN["thickness"],
                structure_material=link_silver,
                pad_material=pad_black,
                distal=segment_name == "distal",
            )
        )

    model.articulation(
        _finger_joint_name("thumb", "palm", "proximal"),
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_parts[0],
        origin=Origin(
            xyz=(
                0.026,
                THUMB_CHAIN["root_y"],
                THUMB_CHAIN["root_z"],
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.4,
            lower=0.0,
            upper=THUMB_CHAIN["limits"][0],
        ),
    )
    for index in range(2):
        model.articulation(
            _finger_joint_name("thumb", LINK_SEGMENT_NAMES[index], LINK_SEGMENT_NAMES[index + 1]),
            ArticulationType.REVOLUTE,
            parent=thumb_parts[index],
            child=thumb_parts[index + 1],
            origin=Origin(xyz=(THUMB_CHAIN["lengths"][index], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.8,
                lower=0.0,
                upper=THUMB_CHAIN["limits"][index + 1],
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")

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

    finger_root_positions: list[tuple[str, tuple[float, float, float]]] = []
    all_joint_names: list[str] = []

    for chain in FINGER_CHAINS:
        segment_parts = [
            object_model.get_part(_link_part_name(chain["name"], segment_name))
            for segment_name in LINK_SEGMENT_NAMES
        ]

        root_joint_name = _finger_joint_name(chain["name"], "palm", "proximal")
        root_joint = object_model.get_articulation(root_joint_name)
        all_joint_names.append(root_joint_name)
        ctx.check(
            f"{root_joint_name}_axis",
            tuple(root_joint.axis) == (0.0, 1.0, 0.0),
            details=f"Expected transverse finger axis (0, 1, 0), got {root_joint.axis}",
        )
        ctx.expect_contact(
            palm,
            segment_parts[0],
            contact_tol=1e-5,
            name=f"{chain['name']}_root_contact",
        )

        root_position = ctx.part_world_position(segment_parts[0])
        if root_position is None:
            ctx.fail(f"{chain['name']}_root_position", "Finger root has no world position.")
        else:
            finger_root_positions.append((chain["name"], root_position))
            ctx.check(
                f"{chain['name']}_root_forward_of_palm",
                root_position[0] > 0.025,
                details=f"Expected {chain['name']} root ahead of palm face, got x={root_position[0]:.4f}",
            )

        for index in range(2):
            joint_name = _finger_joint_name(
                chain["name"],
                LINK_SEGMENT_NAMES[index],
                LINK_SEGMENT_NAMES[index + 1],
            )
            all_joint_names.append(joint_name)
            joint = object_model.get_articulation(joint_name)
            ctx.check(
                f"{joint_name}_axis",
                tuple(joint.axis) == (0.0, 1.0, 0.0),
                details=f"Expected transverse finger axis (0, 1, 0), got {joint.axis}",
            )
            ctx.expect_contact(
                segment_parts[index],
                segment_parts[index + 1],
                contact_tol=1e-5,
                name=f"{joint_name}_contact",
            )

    finger_root_positions.sort(key=lambda item: item[1][1])
    ctx.check(
        "finger_root_y_order",
        [name for name, _ in finger_root_positions] == ["little", "ring", "middle", "index"],
        details=f"Unexpected finger root lateral order: {[name for name, _ in finger_root_positions]}",
    )

    thumb_parts = [
        object_model.get_part(_link_part_name("thumb", segment_name))
        for segment_name in LINK_SEGMENT_NAMES
    ]
    thumb_root_joint_name = _finger_joint_name("thumb", "palm", "proximal")
    thumb_root_joint = object_model.get_articulation(thumb_root_joint_name)
    all_joint_names.append(thumb_root_joint_name)
    ctx.check(
        f"{thumb_root_joint_name}_axis",
        tuple(thumb_root_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Expected thumb axis (0, 0, 1), got {thumb_root_joint.axis}",
    )
    ctx.expect_contact(palm, thumb_parts[0], contact_tol=1e-5, name="thumb_root_contact")

    thumb_root_position = ctx.part_world_position(thumb_parts[0])
    index_root_position = next(position for name, position in finger_root_positions if name == "index")
    if thumb_root_position is None:
        ctx.fail("thumb_root_position", "Thumb root has no world position.")
    else:
        ctx.check(
            "thumb_is_side_mounted",
            thumb_root_position[1] > index_root_position[1] + 0.020,
            details=(
                "Thumb should mount laterally outside the finger row; "
                f"thumb y={thumb_root_position[1]:.4f}, index y={index_root_position[1]:.4f}"
            ),
        )
        ctx.check(
            "thumb_is_lower_than_finger_row",
            thumb_root_position[2] < min(position[2] for _, position in finger_root_positions) - 0.020,
            details=f"Thumb z={thumb_root_position[2]:.4f} should sit below the finger roots.",
        )

    for index in range(2):
        joint_name = _finger_joint_name("thumb", LINK_SEGMENT_NAMES[index], LINK_SEGMENT_NAMES[index + 1])
        joint = object_model.get_articulation(joint_name)
        all_joint_names.append(joint_name)
        ctx.check(
            f"{joint_name}_axis",
            tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"Expected thumb axis (0, 0, 1), got {joint.axis}",
        )
        ctx.expect_contact(
            thumb_parts[index],
            thumb_parts[index + 1],
            contact_tol=1e-5,
            name=f"{joint_name}_contact",
        )

    for joint_name in all_joint_names:
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_has_limits",
            limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper > 0.7,
            details=f"Joint {joint_name} should have bounded flexion limits, got {limits}",
        )
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint_name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint_name}_upper_no_floating")
                parent_part = object_model.get_part(joint.parent)
                child_part = object_model.get_part(joint.child)
                ctx.expect_contact(
                    parent_part,
                    child_part,
                    contact_tol=1e-5,
                    name=f"{joint_name}_upper_contact",
                )

    curled_pose = {}
    for chain in FINGER_CHAINS:
        curled_pose[object_model.get_articulation(_finger_joint_name(chain["name"], "palm", "proximal"))] = 0.72 * chain["limits"][0]
        curled_pose[object_model.get_articulation(_finger_joint_name(chain["name"], "proximal", "middle"))] = 0.72 * chain["limits"][1]
        curled_pose[object_model.get_articulation(_finger_joint_name(chain["name"], "middle", "distal"))] = 0.72 * chain["limits"][2]
    curled_pose[object_model.get_articulation(_finger_joint_name("thumb", "palm", "proximal"))] = 0.70 * THUMB_CHAIN["limits"][0]
    curled_pose[object_model.get_articulation(_finger_joint_name("thumb", "proximal", "middle"))] = 0.70 * THUMB_CHAIN["limits"][1]
    curled_pose[object_model.get_articulation(_finger_joint_name("thumb", "middle", "distal"))] = 0.70 * THUMB_CHAIN["limits"][2]
    with ctx.pose(curled_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="curled_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="curled_pose_no_floating")

    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
