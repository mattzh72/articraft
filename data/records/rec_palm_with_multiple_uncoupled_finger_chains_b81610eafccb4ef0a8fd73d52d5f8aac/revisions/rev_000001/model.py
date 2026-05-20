from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PALM_WIDTH = 0.118
PALM_DEPTH = 0.082
PALM_HEIGHT = 0.052

FINGER_SLOT_DEPTH = 0.010
FINGER_EAR_THICKNESS = 0.0035
FINGER_SHOULDER_DEPTH = 0.006
FINGER_ROOT_Y = PALM_DEPTH * 0.5 + FINGER_SLOT_DEPTH * 0.5

THUMB_SLOT_DEPTH = 0.012
THUMB_EAR_THICKNESS = 0.0035
THUMB_SHOULDER_DEPTH = 0.007
THUMB_ROOT_X = -(PALM_WIDTH * 0.5 + 0.041)
THUMB_BASE_INNER_X = THUMB_ROOT_X + THUMB_SLOT_DEPTH * 0.5
THUMB_BRIDGE_INNER_X = -(PALM_WIDTH * 0.5)
THUMB_BRIDGE_WIDTH = THUMB_BRIDGE_INNER_X - THUMB_BASE_INNER_X
THUMB_BRIDGE_CENTER_X = (THUMB_BRIDGE_INNER_X + THUMB_BASE_INNER_X) * 0.5

FINGER_SPECS = (
    {
        "name": "index",
        "x": -0.036,
        "z": 0.032,
        "base_w": 0.0140,
        "mid_w": 0.0124,
        "dist_w": 0.0104,
        "lengths": (0.041, 0.029, 0.022),
        "thicknesses": (0.018, 0.016, 0.014),
    },
    {
        "name": "middle",
        "x": -0.012,
        "z": 0.034,
        "base_w": 0.0150,
        "mid_w": 0.0130,
        "dist_w": 0.0110,
        "lengths": (0.046, 0.032, 0.024),
        "thicknesses": (0.019, 0.017, 0.015),
    },
    {
        "name": "ring",
        "x": 0.012,
        "z": 0.033,
        "base_w": 0.0148,
        "mid_w": 0.0128,
        "dist_w": 0.0108,
        "lengths": (0.043, 0.030, 0.023),
        "thicknesses": (0.018, 0.0165, 0.0145),
    },
    {
        "name": "little",
        "x": 0.036,
        "z": 0.030,
        "base_w": 0.0130,
        "mid_w": 0.0115,
        "dist_w": 0.0095,
        "lengths": (0.034, 0.025, 0.019),
        "thicknesses": (0.0165, 0.0145, 0.013),
    },
)

THUMB_SPEC = {
    "name": "thumb",
    "y": 0.052,
    "z": 0.017,
    "base_span": 0.020,
    "mid_span": 0.017,
    "dist_span": 0.014,
    "lengths": (0.052, 0.036, 0.028),
    "thicknesses": (0.017, 0.015, 0.0135),
}


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _build_finger_segment(
    part,
    *,
    base_width: float,
    thickness: float,
    length: float,
    next_base_width: float | None,
    shell_material,
    knuckle_material,
    pad_material,
) -> None:
    _add_box(
        part,
        size=(base_width, FINGER_SLOT_DEPTH, thickness),
        xyz=(0.0, 0.0, 0.0),
        material=knuckle_material,
        name="base_block",
    )
    _add_box(
        part,
        size=(base_width * 0.72, FINGER_SLOT_DEPTH * 0.62, thickness * 0.20),
        xyz=(0.0, 0.0, thickness * 0.40),
        material=shell_material,
        name="knuckle_cap",
    )

    if next_base_width is None:
        tip_depth = 0.012
        body_end = max(0.010, length - tip_depth)
        _add_box(
            part,
            size=(base_width * 0.88, body_end, thickness * 0.84),
            xyz=(0.0, body_end * 0.5, 0.0),
            material=shell_material,
            name="body",
        )
        _add_box(
            part,
            size=(base_width * 0.78, tip_depth, thickness * 0.70),
            xyz=(0.0, body_end + tip_depth * 0.5, -thickness * 0.06),
            material=pad_material,
            name="tip_pad",
        )
        return

    outer_width = next_base_width + 2.0 * FINGER_EAR_THICKNESS
    shoulder_back = length - FINGER_SLOT_DEPTH * 0.5 - FINGER_SHOULDER_DEPTH
    body_width = max(base_width * 0.88, outer_width * 0.78)

    _add_box(
        part,
        size=(body_width, shoulder_back, thickness * 0.84),
        xyz=(0.0, shoulder_back * 0.5, 0.0),
        material=shell_material,
        name="body",
    )
    _add_box(
        part,
        size=(outer_width, FINGER_SHOULDER_DEPTH, thickness * 0.90),
        xyz=(0.0, shoulder_back + FINGER_SHOULDER_DEPTH * 0.5, 0.0),
        material=knuckle_material,
        name="distal_shoulder",
    )
    for sign, label in ((-1.0, "left"), (1.0, "right")):
        _add_box(
            part,
            size=(FINGER_EAR_THICKNESS, FINGER_SLOT_DEPTH, thickness),
            xyz=(
                sign * (next_base_width * 0.5 + FINGER_EAR_THICKNESS * 0.5),
                length,
                0.0,
            ),
            material=knuckle_material,
            name=f"{label}_clevis",
        )


def _build_thumb_segment(
    part,
    *,
    base_span: float,
    thickness: float,
    length: float,
    next_base_span: float | None,
    shell_material,
    knuckle_material,
    pad_material,
) -> None:
    _add_box(
        part,
        size=(THUMB_SLOT_DEPTH, base_span, thickness),
        xyz=(0.0, 0.0, 0.0),
        material=knuckle_material,
        name="base_block",
    )
    _add_box(
        part,
        size=(THUMB_SLOT_DEPTH * 0.62, base_span * 0.72, thickness * 0.20),
        xyz=(0.0, 0.0, thickness * 0.38),
        material=shell_material,
        name="knuckle_cap",
    )

    if next_base_span is None:
        tip_depth = 0.013
        body_end = max(0.012, length - tip_depth)
        _add_box(
            part,
            size=(body_end, base_span * 0.86, thickness * 0.84),
            xyz=(body_end * 0.5, 0.0, 0.0),
            material=shell_material,
            name="body",
        )
        _add_box(
            part,
            size=(tip_depth, base_span * 0.76, thickness * 0.68),
            xyz=(body_end + tip_depth * 0.5, 0.0, -thickness * 0.06),
            material=pad_material,
            name="tip_pad",
        )
        return

    outer_span = next_base_span + 2.0 * THUMB_EAR_THICKNESS
    shoulder_back = length - THUMB_SLOT_DEPTH * 0.5 - THUMB_SHOULDER_DEPTH
    body_span = max(base_span * 0.88, outer_span * 0.78)

    _add_box(
        part,
        size=(shoulder_back, body_span, thickness * 0.84),
        xyz=(shoulder_back * 0.5, 0.0, 0.0),
        material=shell_material,
        name="body",
    )
    _add_box(
        part,
        size=(THUMB_SHOULDER_DEPTH, outer_span, thickness * 0.90),
        xyz=(shoulder_back + THUMB_SHOULDER_DEPTH * 0.5, 0.0, 0.0),
        material=knuckle_material,
        name="distal_shoulder",
    )
    for sign, label in ((-1.0, "rear"), (1.0, "front")):
        _add_box(
            part,
            size=(THUMB_SLOT_DEPTH, THUMB_EAR_THICKNESS, thickness),
            xyz=(
                length,
                sign * (next_base_span * 0.5 + THUMB_EAR_THICKNESS * 0.5),
                0.0,
            ),
            material=knuckle_material,
            name=f"{label}_clevis",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dexterous_palm")

    palm_shell = model.material("palm_shell", rgba=(0.18, 0.19, 0.22, 1.0))
    palm_cover = model.material("palm_cover", rgba=(0.26, 0.28, 0.31, 1.0))
    digit_shell = model.material("digit_shell", rgba=(0.61, 0.63, 0.67, 1.0))
    digit_knuckle = model.material("digit_knuckle", rgba=(0.48, 0.50, 0.54, 1.0))
    grip_pad = model.material("grip_pad", rgba=(0.12, 0.12, 0.13, 1.0))

    palm = model.part("palm")
    _add_box(
        palm,
        size=(PALM_WIDTH, PALM_DEPTH, 0.040),
        xyz=(0.0, 0.0, 0.020),
        material=palm_shell,
        name="palm_core",
    )
    _add_box(
        palm,
        size=(0.096, 0.056, 0.012),
        xyz=(0.0, -0.004, 0.046),
        material=palm_cover,
        name="dorsal_cover",
    )
    _add_box(
        palm,
        size=(0.026, 0.034, 0.022),
        xyz=(-0.046, 0.024, 0.017),
        material=palm_cover,
        name="thumb_mount_block",
    )

    for spec in FINGER_SPECS:
        ear_height = spec["thicknesses"][0] + 0.010
        for sign, label in ((-1.0, "left"), (1.0, "right")):
            _add_box(
                palm,
                size=(FINGER_EAR_THICKNESS, FINGER_SLOT_DEPTH, ear_height),
                xyz=(
                    spec["x"] + sign * (spec["base_w"] * 0.5 + FINGER_EAR_THICKNESS * 0.5),
                    FINGER_ROOT_Y,
                    spec["z"],
                ),
                material=palm_shell,
                name=f"{spec['name']}_{label}_mount",
            )

    thumb_outer_span = THUMB_SPEC["base_span"] + 2.0 * THUMB_EAR_THICKNESS
    for sign, label in ((-1.0, "rear"), (1.0, "front")):
        _add_box(
            palm,
            size=(THUMB_SLOT_DEPTH, THUMB_EAR_THICKNESS, THUMB_SPEC["thicknesses"][0] + 0.008),
            xyz=(
                THUMB_ROOT_X,
                THUMB_SPEC["y"] + sign * (THUMB_SPEC["base_span"] * 0.5 + THUMB_EAR_THICKNESS * 0.5),
                THUMB_SPEC["z"],
            ),
            material=palm_shell,
            name=f"thumb_{label}_mount",
        )
    _add_box(
        palm,
        size=(THUMB_BRIDGE_WIDTH, thumb_outer_span, 0.008),
        xyz=(THUMB_BRIDGE_CENTER_X, THUMB_SPEC["y"], 0.002),
        material=palm_cover,
        name="thumb_mount_bridge",
    )

    for spec in FINGER_SPECS:
        proximal = model.part(f"{spec['name']}_proximal")
        middle = model.part(f"{spec['name']}_middle")
        distal = model.part(f"{spec['name']}_distal")

        _build_finger_segment(
            proximal,
            base_width=spec["base_w"],
            thickness=spec["thicknesses"][0],
            length=spec["lengths"][0],
            next_base_width=spec["mid_w"],
            shell_material=digit_shell,
            knuckle_material=digit_knuckle,
            pad_material=grip_pad,
        )
        _build_finger_segment(
            middle,
            base_width=spec["mid_w"],
            thickness=spec["thicknesses"][1],
            length=spec["lengths"][1],
            next_base_width=spec["dist_w"],
            shell_material=digit_shell,
            knuckle_material=digit_knuckle,
            pad_material=grip_pad,
        )
        _build_finger_segment(
            distal,
            base_width=spec["dist_w"],
            thickness=spec["thicknesses"][2],
            length=spec["lengths"][2],
            next_base_width=None,
            shell_material=digit_shell,
            knuckle_material=digit_knuckle,
            pad_material=grip_pad,
        )

        model.articulation(
            f"palm_to_{spec['name']}",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(spec["x"], FINGER_ROOT_Y, spec["z"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=14.0,
                velocity=4.0,
                lower=-0.18,
                upper=1.35,
            ),
        )
        model.articulation(
            f"{spec['name']}_proximal_to_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, spec["lengths"][0], 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=5.0,
                lower=0.0,
                upper=1.55,
            ),
        )
        model.articulation(
            f"{spec['name']}_middle_to_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, spec["lengths"][1], 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=5.0,
                lower=0.0,
                upper=1.35,
            ),
        )

    thumb_proximal = model.part("thumb_proximal")
    thumb_middle = model.part("thumb_middle")
    thumb_distal = model.part("thumb_distal")

    _build_thumb_segment(
        thumb_proximal,
        base_span=THUMB_SPEC["base_span"],
        thickness=THUMB_SPEC["thicknesses"][0],
        length=THUMB_SPEC["lengths"][0],
        next_base_span=THUMB_SPEC["mid_span"],
        shell_material=digit_shell,
        knuckle_material=digit_knuckle,
        pad_material=grip_pad,
    )
    _build_thumb_segment(
        thumb_middle,
        base_span=THUMB_SPEC["mid_span"],
        thickness=THUMB_SPEC["thicknesses"][1],
        length=THUMB_SPEC["lengths"][1],
        next_base_span=THUMB_SPEC["dist_span"],
        shell_material=digit_shell,
        knuckle_material=digit_knuckle,
        pad_material=grip_pad,
    )
    _build_thumb_segment(
        thumb_distal,
        base_span=THUMB_SPEC["dist_span"],
        thickness=THUMB_SPEC["thicknesses"][2],
        length=THUMB_SPEC["lengths"][2],
        next_base_span=None,
        shell_material=digit_shell,
        knuckle_material=digit_knuckle,
        pad_material=grip_pad,
    )

    model.articulation(
        "palm_to_thumb",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_proximal,
        origin=Origin(xyz=(THUMB_ROOT_X, THUMB_SPEC["y"], THUMB_SPEC["z"])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=4.0,
            lower=-0.45,
            upper=1.05,
        ),
    )
    model.articulation(
        "thumb_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=thumb_proximal,
        child=thumb_middle,
        origin=Origin(xyz=(THUMB_SPEC["lengths"][0], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=5.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "thumb_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=thumb_middle,
        child=thumb_distal,
        origin=Origin(xyz=(THUMB_SPEC["lengths"][1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=5.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    palm = object_model.get_part("palm")
    finger_root_joints = []
    finger_chain_pairs: list[tuple[object, object, str]] = []
    finger_axes_ok = True
    front_edge_ok = True

    for spec in FINGER_SPECS:
        proximal = object_model.get_part(f"{spec['name']}_proximal")
        middle = object_model.get_part(f"{spec['name']}_middle")
        distal = object_model.get_part(f"{spec['name']}_distal")

        root_joint = object_model.get_articulation(f"palm_to_{spec['name']}")
        middle_joint = object_model.get_articulation(f"{spec['name']}_proximal_to_middle")
        distal_joint = object_model.get_articulation(f"{spec['name']}_middle_to_distal")

        finger_root_joints.append(root_joint)
        finger_chain_pairs.extend(
            (
                (palm, proximal, f"{spec['name']}_root_contact"),
                (proximal, middle, f"{spec['name']}_middle_contact"),
                (middle, distal, f"{spec['name']}_distal_contact"),
            )
        )
        finger_axes_ok = finger_axes_ok and (
            root_joint.axis == (1.0, 0.0, 0.0)
            and middle_joint.axis == (1.0, 0.0, 0.0)
            and distal_joint.axis == (1.0, 0.0, 0.0)
        )
        front_edge_ok = front_edge_ok and abs(root_joint.origin.xyz[1] - FINGER_ROOT_Y) < 1e-9

    thumb_proximal = object_model.get_part("thumb_proximal")
    thumb_middle = object_model.get_part("thumb_middle")
    thumb_distal = object_model.get_part("thumb_distal")
    thumb_root = object_model.get_articulation("palm_to_thumb")
    thumb_middle_joint = object_model.get_articulation("thumb_proximal_to_middle")
    thumb_distal_joint = object_model.get_articulation("thumb_middle_to_distal")

    for parent, child, check_name in finger_chain_pairs:
        ctx.expect_contact(parent, child, name=check_name)

    ctx.expect_contact(palm, thumb_proximal, name="thumb_root_contact")
    ctx.expect_contact(thumb_proximal, thumb_middle, name="thumb_middle_contact")
    ctx.expect_contact(thumb_middle, thumb_distal, name="thumb_distal_contact")

    root_xs = [joint.origin.xyz[0] for joint in finger_root_joints]
    ctx.check(
        "finger_root_order",
        root_xs == sorted(root_xs),
        details=f"Finger root x positions not ordered left-to-right: {root_xs}",
    )
    ctx.check(
        "finger_roots_on_front_edge",
        front_edge_ok,
        details="Finger root joints must share the same front-edge mount line.",
    )
    ctx.check(
        "finger_joint_axes",
        finger_axes_ok,
        details="Every finger joint should hinge on the local x-axis.",
    )
    ctx.check(
        "thumb_joint_axes",
        thumb_root.axis == (0.0, 1.0, 0.0)
        and thumb_middle_joint.axis == (0.0, 1.0, 0.0)
        and thumb_distal_joint.axis == (0.0, 1.0, 0.0),
        details="Thumb joints should hinge on the local y-axis for side opposition.",
    )
    ctx.check(
        "thumb_side_mount",
        thumb_root.origin.xyz[0] < -(PALM_WIDTH * 0.5)
        and thumb_root.origin.xyz[1] > (PALM_DEPTH * 0.5),
        details="Thumb root should sit off the palm side and slightly ahead of the front edge.",
    )

    ctx.expect_origin_distance(
        "index_proximal",
        "middle_proximal",
        axes="x",
        min_dist=0.020,
        name="index_middle_spacing",
    )
    ctx.expect_origin_distance(
        "middle_proximal",
        "ring_proximal",
        axes="x",
        min_dist=0.020,
        name="middle_ring_spacing",
    )
    ctx.expect_origin_distance(
        "ring_proximal",
        "little_proximal",
        axes="x",
        min_dist=0.020,
        name="ring_little_spacing",
    )

    index_root = object_model.get_articulation("palm_to_index")
    index_middle_joint = object_model.get_articulation("index_proximal_to_middle")
    index_distal_joint = object_model.get_articulation("index_middle_to_distal")

    with ctx.pose(
        {
            index_root: 0.72,
            index_middle_joint: 0.96,
            index_distal_joint: 0.82,
            thumb_root: 0.58,
            thumb_middle_joint: 0.82,
            thumb_distal_joint: 0.66,
        }
    ):
        ctx.expect_contact(palm, object_model.get_part("index_proximal"), name="index_root_contact_curled")
        ctx.expect_contact(
            object_model.get_part("index_proximal"),
            object_model.get_part("index_middle"),
            name="index_middle_contact_curled",
        )
        ctx.expect_contact(
            object_model.get_part("index_middle"),
            object_model.get_part("index_distal"),
            name="index_distal_contact_curled",
        )
        ctx.expect_contact(palm, thumb_proximal, name="thumb_root_contact_curled")
        ctx.expect_contact(thumb_proximal, thumb_middle, name="thumb_middle_contact_curled")
        ctx.expect_contact(thumb_middle, thumb_distal, name="thumb_distal_contact_curled")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
