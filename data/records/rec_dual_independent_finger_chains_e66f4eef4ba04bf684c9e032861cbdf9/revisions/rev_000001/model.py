from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PALM_TOP_Z = 0.080
PAD_Y = -0.055
NIB_Y = 0.055
PEDESTAL_X = 0.060
PEDESTAL_HINGE_X = 0.052
PEDESTAL_HINGE_Z = 0.065

PAD_LENGTHS = (0.125, 0.105, 0.085)
NIB_LENGTHS = (0.105, 0.082, 0.062)
FINGER_AXIS = (0.0, 1.0, 0.0)
JOINT_LIMITS = MotionLimits(effort=12.0, velocity=2.8, lower=0.0, upper=1.25)


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder helper whose long axis lies along local Y."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _add_pedestal(part, *, material: Material, metal: Material) -> None:
    part.visual(
        Box((0.078, 0.050, 0.020)),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=material,
        name="pedestal_foot",
    )
    part.visual(
        Box((0.032, 0.020, 0.060)),
        origin=Origin(xyz=(0.016, 0.000, 0.047)),
        material=material,
        name="center_web",
    )
    for sign, name in ((-1.0, "side_plate_0"), (1.0, "side_plate_1")):
        part.visual(
            Box((0.054, 0.006, 0.074)),
            origin=Origin(xyz=(0.036, sign * 0.023, 0.054)),
            material=material,
            name=name,
        )
    # Small external pin caps avoid filling the moving barrel bore while still
    # showing a fastener at each side plate.
    cyl, rot = _cyl_y(0.004, 0.006)
    for sign, name in ((-1.0, "pin_cap_0"), (1.0, "pin_cap_1")):
        part.visual(
            cyl,
            origin=Origin(xyz=(PEDESTAL_HINGE_X, sign * 0.028, PEDESTAL_HINGE_Z), rpy=rot.rpy),
            material=metal,
            name=name,
        )


def _add_phalanx(
    part,
    *,
    length: float,
    material: Material,
    metal: Material,
    rubber: Material | None = None,
    distal_clevis: bool,
    rounded_pad: bool = False,
    flat_nib: bool = False,
) -> None:
    outer_y = 0.048
    body_y = 0.024
    body_z = 0.022
    barrel_radius = 0.013

    cyl, rot = _cyl_y(barrel_radius, 0.040)
    part.visual(
        cyl,
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=rot.rpy),
        material=metal,
        name="proximal_barrel",
    )
    part.visual(
        Box((length - 0.032, body_y, body_z)),
        origin=Origin(xyz=(0.012 + (length - 0.032) / 2.0, 0.000, 0.000)),
        material=material,
        name="phalanx_web",
    )
    part.visual(
        Box((0.026, outer_y, 0.018)),
        origin=Origin(xyz=(length - 0.032, 0.000, 0.000)),
        material=material,
        name="distal_bridge",
    )

    if distal_clevis:
        for sign, name in ((-1.0, "distal_plate_0"), (1.0, "distal_plate_1")):
            part.visual(
                Box((0.052, 0.006, 0.034)),
                origin=Origin(xyz=(length - 0.004, sign * 0.023, 0.000)),
                material=material,
                name=name,
            )

    if rounded_pad:
        pad_mat = rubber if rubber is not None else material
        part.visual(
            Box((0.046, 0.026, 0.018)),
            origin=Origin(xyz=(length + 0.001, 0.000, 0.000)),
            material=material,
            name="pad_neck",
        )
        pad_cyl, pad_rot = _cyl_y(0.018, 0.038)
        part.visual(
            pad_cyl,
            origin=Origin(xyz=(length + 0.028, 0.000, 0.000), rpy=pad_rot.rpy),
            material=pad_mat,
            name="rounded_pad",
        )

    if flat_nib:
        nib_mat = rubber if rubber is not None else material
        part.visual(
            Box((0.046, 0.024, 0.014)),
            origin=Origin(xyz=(length + 0.001, 0.000, 0.000)),
            material=material,
            name="nib_neck",
        )
        part.visual(
            Box((0.034, 0.034, 0.010)),
            origin=Origin(xyz=(length + 0.026, 0.000, -0.002)),
            material=nib_mat,
            name="flat_nib",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_finger_manipulator_core")

    graphite = model.material("graphite_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    dark = model.material("dark_split_shadow", rgba=(0.015, 0.016, 0.018, 1.0))
    blue = model.material("blue_hardcoat", rgba=(0.05, 0.17, 0.34, 1.0))
    olive = model.material("olive_hardcoat", rgba=(0.18, 0.23, 0.13, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))

    palm = model.part("split_palm")
    palm.visual(
        Box((0.160, 0.075, 0.080)),
        origin=Origin(xyz=(0.000, PAD_Y, PALM_TOP_Z / 2.0)),
        material=graphite,
        name="palm_lobe_0",
    )
    palm.visual(
        Box((0.160, 0.075, 0.080)),
        origin=Origin(xyz=(0.000, NIB_Y, PALM_TOP_Z / 2.0)),
        material=graphite,
        name="palm_lobe_1",
    )
    palm.visual(
        Box((0.058, 0.172, 0.080)),
        origin=Origin(xyz=(-0.057, 0.000, PALM_TOP_Z / 2.0)),
        material=graphite,
        name="rear_spine",
    )
    palm.visual(
        Box((0.094, 0.010, 0.076)),
        origin=Origin(xyz=(0.032, 0.000, PALM_TOP_Z / 2.0)),
        material=dark,
        name="split_recess",
    )
    palm.visual(
        Box((0.150, 0.185, 0.010)),
        origin=Origin(xyz=(-0.005, 0.000, 0.005)),
        material=dark,
        name="bottom_tie_plate",
    )

    pad_pedestal = model.part("pad_pedestal")
    nib_pedestal = model.part("nib_pedestal")
    _add_pedestal(pad_pedestal, material=blue, metal=steel)
    _add_pedestal(nib_pedestal, material=olive, metal=steel)

    model.articulation(
        "palm_to_pad_pedestal",
        ArticulationType.FIXED,
        parent=palm,
        child=pad_pedestal,
        origin=Origin(xyz=(PEDESTAL_X, PAD_Y, PALM_TOP_Z)),
    )
    model.articulation(
        "palm_to_nib_pedestal",
        ArticulationType.FIXED,
        parent=palm,
        child=nib_pedestal,
        origin=Origin(xyz=(PEDESTAL_X, NIB_Y, PALM_TOP_Z)),
    )

    pad_base = model.part("pad_base")
    pad_middle = model.part("pad_middle")
    pad_distal = model.part("pad_distal")
    nib_base = model.part("nib_base")
    nib_middle = model.part("nib_middle")
    nib_distal = model.part("nib_distal")

    _add_phalanx(pad_base, length=PAD_LENGTHS[0], material=blue, metal=steel, distal_clevis=True)
    _add_phalanx(pad_middle, length=PAD_LENGTHS[1], material=blue, metal=steel, distal_clevis=True)
    _add_phalanx(
        pad_distal,
        length=PAD_LENGTHS[2],
        material=blue,
        metal=steel,
        rubber=rubber,
        distal_clevis=False,
        rounded_pad=True,
    )

    _add_phalanx(nib_base, length=NIB_LENGTHS[0], material=olive, metal=steel, distal_clevis=True)
    _add_phalanx(nib_middle, length=NIB_LENGTHS[1], material=olive, metal=steel, distal_clevis=True)
    _add_phalanx(
        nib_distal,
        length=NIB_LENGTHS[2],
        material=olive,
        metal=steel,
        rubber=rubber,
        distal_clevis=False,
        flat_nib=True,
    )

    model.articulation(
        "pad_base_joint",
        ArticulationType.REVOLUTE,
        parent=pad_pedestal,
        child=pad_base,
        origin=Origin(xyz=(PEDESTAL_HINGE_X, 0.000, PEDESTAL_HINGE_Z)),
        axis=FINGER_AXIS,
        motion_limits=JOINT_LIMITS,
    )
    model.articulation(
        "pad_middle_joint",
        ArticulationType.REVOLUTE,
        parent=pad_base,
        child=pad_middle,
        origin=Origin(xyz=(PAD_LENGTHS[0], 0.000, 0.000)),
        axis=FINGER_AXIS,
        motion_limits=JOINT_LIMITS,
    )
    model.articulation(
        "pad_distal_joint",
        ArticulationType.REVOLUTE,
        parent=pad_middle,
        child=pad_distal,
        origin=Origin(xyz=(PAD_LENGTHS[1], 0.000, 0.000)),
        axis=FINGER_AXIS,
        motion_limits=JOINT_LIMITS,
    )

    model.articulation(
        "nib_base_joint",
        ArticulationType.REVOLUTE,
        parent=nib_pedestal,
        child=nib_base,
        origin=Origin(xyz=(PEDESTAL_HINGE_X, 0.000, PEDESTAL_HINGE_Z)),
        axis=FINGER_AXIS,
        motion_limits=JOINT_LIMITS,
    )
    model.articulation(
        "nib_middle_joint",
        ArticulationType.REVOLUTE,
        parent=nib_base,
        child=nib_middle,
        origin=Origin(xyz=(NIB_LENGTHS[0], 0.000, 0.000)),
        axis=FINGER_AXIS,
        motion_limits=JOINT_LIMITS,
    )
    model.articulation(
        "nib_distal_joint",
        ArticulationType.REVOLUTE,
        parent=nib_middle,
        child=nib_distal,
        origin=Origin(xyz=(NIB_LENGTHS[1], 0.000, 0.000)),
        axis=FINGER_AXIS,
        motion_limits=JOINT_LIMITS,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("split_palm")
    pad_pedestal = object_model.get_part("pad_pedestal")
    nib_pedestal = object_model.get_part("nib_pedestal")
    pad_distal = object_model.get_part("pad_distal")
    nib_distal = object_model.get_part("nib_distal")

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "six independent revolute joints",
        len(revolute_joints) == 6 and all(joint.mimic is None for joint in revolute_joints),
        details=f"revolute_joints={[joint.name for joint in revolute_joints]}",
    )
    ctx.check(
        "two side-by-side pedestal mounts",
        {joint.parent for joint in object_model.articulations if joint.name.startswith("palm_to_")}
        == {"split_palm"},
        details="Both fixed pedestal mounts should attach directly to the split palm.",
    )
    ctx.expect_contact(
        pad_pedestal,
        palm,
        elem_a="pedestal_foot",
        name="pad pedestal foot sits on palm",
    )
    ctx.expect_contact(
        nib_pedestal,
        palm,
        elem_a="pedestal_foot",
        name="nib pedestal foot sits on palm",
    )

    ctx.expect_origin_distance(
        pad_distal,
        nib_distal,
        axes="y",
        min_dist=0.080,
        name="finger chains remain side by side",
    )
    ctx.expect_overlap(
        pad_pedestal,
        nib_pedestal,
        axes="x",
        min_overlap=0.050,
        elem_a="pedestal_foot",
        elem_b="pedestal_foot",
        name="separate pedestals share the palm front line",
    )
    ctx.expect_gap(
        nib_pedestal,
        pad_pedestal,
        axis="y",
        min_gap=0.050,
        positive_elem="pedestal_foot",
        negative_elem="pedestal_foot",
        name="separate pedestals have a visible side gap",
    )

    nib_rest = ctx.part_world_position(nib_distal)
    with ctx.pose({"pad_base_joint": 0.55, "pad_middle_joint": 0.65, "pad_distal_joint": 0.45}):
        pad_curled = ctx.part_world_position(pad_distal)
        nib_after = ctx.part_world_position(nib_distal)

    ctx.check(
        "pad chain bends without driving nib chain",
        pad_curled is not None
        and nib_rest is not None
        and nib_after is not None
        and pad_curled[2] < PALM_TOP_Z + PEDESTAL_HINGE_Z - 0.020
        and abs(nib_after[0] - nib_rest[0]) < 1.0e-6
        and abs(nib_after[1] - nib_rest[1]) < 1.0e-6
        and abs(nib_after[2] - nib_rest[2]) < 1.0e-6,
        details=f"pad_curled={pad_curled}, nib_rest={nib_rest}, nib_after={nib_after}",
    )

    return ctx.report()


object_model = build_object_model()
