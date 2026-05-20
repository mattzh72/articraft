from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


Y_AXIS_ORIGIN = Origin(rpy=(-pi / 2.0, 0.0, 0.0))


def _cyl_y(part, *, name: str, radius: float, length: float, xyz, material: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=Y_AXIS_ORIGIN.rpy),
        material=material,
        name=name,
    )


def _top_screw(part, *, name: str, x: float, y: float, z_top: float, material: str) -> None:
    part.visual(
        Cylinder(radius=0.0042, length=0.0022),
        # Slightly seated into the host surface so the head reads as installed.
        origin=Origin(xyz=(x, y, z_top + 0.0001)),
        material=material,
        name=name,
    )


def _side_screw(part, *, name: str, x: float, y: float, z: float, material: str) -> None:
    _cyl_y(
        part,
        name=name,
        radius=0.0036,
        length=0.0030,
        xyz=(x, y, z),
        material=material,
    )


def _add_palm(model: ArticulatedObject):
    palm = model.part("palm")
    palm.visual(
        Box((0.120, 0.170, 0.024)),
        origin=Origin(xyz=(-0.035, 0.0, 0.012)),
        material="dark_anodized",
        name="rear_bridge",
    )
    for label, y in (("round", 0.045), ("nib", -0.045)):
        palm_block_name = "round_palm_block" if label == "round" else "nib_palm_block"
        split_shadow_name = "round_split_shadow" if label == "round" else "nib_split_shadow"
        palm.visual(
            Box((0.090, 0.055, 0.050)),
            origin=Origin(xyz=(0.026, y, 0.025)),
            material="palm_aluminum",
            name=palm_block_name,
        )
        palm.visual(
            Box((0.070, 0.004, 0.052)),
            origin=Origin(xyz=(0.034, y - (0.029 if y > 0 else -0.029), 0.026)),
            material="black_gap",
            name=split_shadow_name,
        )
        for i, sx in enumerate((-0.010, 0.067)):
            for j, sy in enumerate((-0.016, 0.016)):
                _top_screw(
                    palm,
                    name=f"{label}_palm_screw_{i}_{j}",
                    x=sx,
                    y=y + sy,
                    z_top=0.050,
                    material="screw_black",
                )
    return palm


def _add_pedestal(model: ArticulatedObject, palm, *, name: str, y: float, material: str):
    pedestal = model.part(name)
    pedestal.visual(
        Box((0.055, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=material,
        name="foot_block",
    )
    pedestal.visual(
        Box((0.025, 0.022, 0.026)),
        origin=Origin(xyz=(-0.004, 0.0, 0.031)),
        material=material,
        name="center_rib",
    )
    pedestal.visual(
        Box((0.042, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=material,
        name="lower_yoke_bridge",
    )
    for side, sy in (("pos", 0.020), ("neg", -0.020)):
        side_plate_name = "side_plate_pos" if side == "pos" else "side_plate_neg"
        pedestal.visual(
            Box((0.040, 0.008, 0.052)),
            origin=Origin(xyz=(0.0, sy, 0.050)),
            material=material,
            name=side_plate_name,
        )
        _cyl_y(
            pedestal,
            name=f"bearing_boss_{side}",
            radius=0.016,
            length=0.006,
            xyz=(0.0, sy + (0.007 if sy > 0 else -0.007), 0.060),
            material="steel_dark",
        )
        for k, sx in enumerate((-0.010, 0.011)):
            _side_screw(
                pedestal,
                name=f"plate_screw_{side}_{k}",
                x=sx,
                y=sy + (0.0055 if sy > 0 else -0.0055),
                z=0.050,
                material="screw_black",
            )
    _cyl_y(
        pedestal,
        name="base_pin",
        radius=0.0055,
        length=0.052,
        xyz=(0.0, 0.0, 0.060),
        material="steel_dark",
    )
    for i, sx in enumerate((-0.016, 0.016)):
        _top_screw(
            pedestal,
            name=f"foot_screw_{i}",
            x=sx,
            y=0.0,
            z_top=0.018,
            material="screw_black",
        )
    model.articulation(
        f"palm_to_{name}",
        ArticulationType.FIXED,
        parent=palm,
        child=pedestal,
        origin=Origin(xyz=(0.035, y, 0.050)),
    )
    return pedestal


def _add_link(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    material: str,
    distal_style: str | None = None,
):
    link = model.part(name)
    link.visual(
        Box((0.028, 0.018, 0.020)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=material,
        name="proximal_lug",
    )
    _cyl_y(
        link,
        name="proximal_barrel",
        radius=0.0135,
        length=0.024,
        xyz=(0.0, 0.0, 0.0),
        material="steel_dark",
    )

    if distal_style is None:
        spar_length = length - 0.041
        link.visual(
            Box((spar_length, 0.018, 0.016)),
            origin=Origin(xyz=(0.006 + spar_length / 2.0, 0.0, 0.0)),
            material=material,
            name="main_spar",
        )
        link.visual(
            Box((0.018, 0.048, 0.012)),
            origin=Origin(xyz=(length - 0.050, 0.0, 0.0)),
            material=material,
            name="yoke_bridge",
        )
        _cyl_y(
            link,
            name="distal_pin",
            radius=0.0055,
            length=0.052,
            xyz=(length, 0.0, 0.0),
            material="steel_dark",
        )
        for side, sy in (("pos", 0.020), ("neg", -0.020)):
            distal_plate_name = "distal_plate_pos" if side == "pos" else "distal_plate_neg"
            link.visual(
                Box((0.058, 0.008, 0.028)),
                origin=Origin(xyz=(length - 0.018, sy, 0.0)),
                material=material,
                name=distal_plate_name,
            )
            for k, sx in enumerate((length - 0.034, length - 0.004)):
                _side_screw(
                    link,
                    name=f"distal_screw_{side}_{k}",
                    x=sx,
                    y=sy + (0.0055 if sy > 0 else -0.0055),
                    z=0.0,
                    material="screw_black",
                )
        _top_screw(
            link,
            name="spar_screw_0",
            x=max(0.034, length * 0.38),
            y=0.0,
            z_top=0.008,
            material="screw_black",
        )
    else:
        spar_length = length - 0.005
        link.visual(
            Box((spar_length, 0.018, 0.016)),
            origin=Origin(xyz=(0.006 + spar_length / 2.0, 0.0, 0.0)),
            material=material,
            name="main_spar",
        )
        link.visual(
            Box((0.032, 0.022, 0.018)),
            origin=Origin(xyz=(length - 0.006, 0.0, 0.0)),
            material=material,
            name="tip_carrier",
        )
        if distal_style == "rounded":
            _cyl_y(
                link,
                name="rounded_pad",
                radius=0.018,
                length=0.030,
                xyz=(length + 0.018, 0.0, 0.0),
                material="rubber_blue",
            )
            link.visual(
                Box((0.006, 0.024, 0.018)),
                origin=Origin(xyz=(length + 0.002, 0.0, 0.0)),
                material="rubber_blue",
                name="pad_insert",
            )
        elif distal_style == "flat":
            link.visual(
                Box((0.036, 0.026, 0.010)),
                origin=Origin(xyz=(length + 0.018, 0.0, -0.002)),
                material="ceramic_nib",
                name="flat_nib",
            )
            link.visual(
                Box((0.010, 0.024, 0.014)),
                origin=Origin(xyz=(length + 0.001, 0.0, -0.001)),
                material="ceramic_nib",
                name="nib_insert",
            )
        _top_screw(
            link,
            name="tip_screw",
            x=length - 0.012,
            y=0.0,
            z_top=0.009,
            material="screw_black",
        )
    return link


def _add_chain(
    model: ArticulatedObject,
    *,
    prefix: str,
    pedestal,
    lengths: tuple[float, float, float],
    distal_style: str,
    material: str,
    base_upper: float,
    mid_upper: float,
    dist_upper: float,
):
    base = _add_link(model, name=f"{prefix}_base", length=lengths[0], material=material)
    middle = _add_link(model, name=f"{prefix}_middle", length=lengths[1], material=material)
    distal = _add_link(
        model,
        name=f"{prefix}_distal",
        length=lengths[2],
        material=material,
        distal_style=distal_style,
    )

    model.articulation(
        f"{prefix}_base_joint",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=base,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=base_upper, effort=18.0, velocity=2.0),
    )
    model.articulation(
        f"{prefix}_middle_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=middle,
        origin=Origin(xyz=(lengths[0], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=mid_upper, effort=12.0, velocity=2.2),
    )
    model.articulation(
        f"{prefix}_distal_joint",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(lengths[1], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=dist_upper, effort=8.0, velocity=2.6),
    )
    return base, middle, distal


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_palm_two_finger_manipulator")
    model.material("palm_aluminum", rgba=(0.54, 0.57, 0.60, 1.0))
    model.material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("finger_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("warm_aluminum", rgba=(0.66, 0.62, 0.54, 1.0))
    model.material("steel_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("screw_black", rgba=(0.01, 0.01, 0.012, 1.0))
    model.material("rubber_blue", rgba=(0.03, 0.09, 0.15, 1.0))
    model.material("ceramic_nib", rgba=(0.86, 0.84, 0.76, 1.0))
    model.material("black_gap", rgba=(0.0, 0.0, 0.0, 1.0))

    palm = _add_palm(model)
    round_pedestal = _add_pedestal(
        model,
        palm,
        name="round_pedestal",
        y=0.045,
        material="dark_anodized",
    )
    nib_pedestal = _add_pedestal(
        model,
        palm,
        name="nib_pedestal",
        y=-0.045,
        material="dark_anodized",
    )
    _add_chain(
        model,
        prefix="round",
        pedestal=round_pedestal,
        lengths=(0.116, 0.088, 0.070),
        distal_style="rounded",
        material="finger_aluminum",
        base_upper=0.70,
        mid_upper=0.78,
        dist_upper=0.62,
    )
    _add_chain(
        model,
        prefix="nib",
        pedestal=nib_pedestal,
        lengths=(0.098, 0.086, 0.056),
        distal_style="flat",
        material="warm_aluminum",
        base_upper=0.60,
        mid_upper=0.70,
        dist_upper=0.58,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")
    round_pedestal = object_model.get_part("round_pedestal")
    nib_pedestal = object_model.get_part("nib_pedestal")
    round_base = object_model.get_part("round_base")
    round_middle = object_model.get_part("round_middle")
    round_distal = object_model.get_part("round_distal")
    nib_base = object_model.get_part("nib_base")
    nib_middle = object_model.get_part("nib_middle")
    nib_distal = object_model.get_part("nib_distal")

    revolute_joints = [
        j for j in object_model.articulations if j.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "six independent revolute joints",
        len(revolute_joints) == 6
        and all(j.mimic is None for j in revolute_joints)
        and {j.name.split("_")[0] for j in revolute_joints} == {"round", "nib"},
        details=f"revolute joints={[j.name for j in revolute_joints]}",
    )
    ctx.expect_contact(
        round_pedestal,
        palm,
        elem_a="foot_block",
        elem_b="round_palm_block",
        name="rounded-finger pedestal is grounded on palm",
    )
    ctx.expect_contact(
        nib_pedestal,
        palm,
        elem_a="foot_block",
        elem_b="nib_palm_block",
        name="nib-finger pedestal is grounded on palm",
    )
    ctx.allow_overlap(
        round_pedestal,
        round_base,
        elem_a="base_pin",
        elem_b="proximal_barrel",
        reason="The exposed base axle is intentionally captured through the round finger root barrel.",
    )
    ctx.expect_overlap(
        round_pedestal,
        round_base,
        axes="xyz",
        elem_a="base_pin",
        elem_b="proximal_barrel",
        min_overlap=0.010,
        name="round root pin passes through barrel",
    )
    ctx.allow_overlap(
        nib_pedestal,
        nib_base,
        elem_a="base_pin",
        elem_b="proximal_barrel",
        reason="The exposed base axle is intentionally captured through the nib finger root barrel.",
    )
    ctx.expect_overlap(
        nib_pedestal,
        nib_base,
        axes="xyz",
        elem_a="base_pin",
        elem_b="proximal_barrel",
        min_overlap=0.010,
        name="nib root pin passes through barrel",
    )
    ctx.allow_overlap(
        round_base,
        round_middle,
        elem_a="distal_pin",
        elem_b="proximal_barrel",
        reason="The round base-link knuckle pin is intentionally seated through the middle-link barrel.",
    )
    ctx.expect_overlap(
        round_base,
        round_middle,
        axes="xyz",
        elem_a="distal_pin",
        elem_b="proximal_barrel",
        min_overlap=0.010,
        name="round middle pin passes through barrel",
    )
    ctx.allow_overlap(
        round_middle,
        round_distal,
        elem_a="distal_pin",
        elem_b="proximal_barrel",
        reason="The round distal knuckle pin is intentionally seated through the distal-link barrel.",
    )
    ctx.expect_overlap(
        round_middle,
        round_distal,
        axes="xyz",
        elem_a="distal_pin",
        elem_b="proximal_barrel",
        min_overlap=0.010,
        name="round distal pin passes through barrel",
    )
    ctx.allow_overlap(
        nib_base,
        nib_middle,
        elem_a="distal_pin",
        elem_b="proximal_barrel",
        reason="The nib base-link knuckle pin is intentionally seated through the middle-link barrel.",
    )
    ctx.expect_overlap(
        nib_base,
        nib_middle,
        axes="xyz",
        elem_a="distal_pin",
        elem_b="proximal_barrel",
        min_overlap=0.010,
        name="nib middle pin passes through barrel",
    )
    ctx.allow_overlap(
        nib_middle,
        nib_distal,
        elem_a="distal_pin",
        elem_b="proximal_barrel",
        reason="The nib distal knuckle pin is intentionally seated through the distal-link barrel.",
    )
    ctx.expect_overlap(
        nib_middle,
        nib_distal,
        axes="xyz",
        elem_a="distal_pin",
        elem_b="proximal_barrel",
        min_overlap=0.010,
        name="nib distal pin passes through barrel",
    )
    ctx.expect_gap(
        round_pedestal,
        round_base,
        axis="y",
        positive_elem="side_plate_pos",
        negative_elem="proximal_barrel",
        min_gap=0.002,
        max_gap=0.008,
        name="round base barrel clears pedestal cheek",
    )
    ctx.expect_gap(
        round_base,
        round_middle,
        axis="y",
        positive_elem="distal_plate_pos",
        negative_elem="proximal_barrel",
        min_gap=0.002,
        max_gap=0.008,
        name="round middle barrel clears base yoke",
    )
    ctx.expect_gap(
        nib_base,
        nib_middle,
        axis="y",
        positive_elem="distal_plate_pos",
        negative_elem="proximal_barrel",
        min_gap=0.002,
        max_gap=0.008,
        name="nib middle barrel clears base yoke",
    )
    ctx.expect_gap(
        round_base,
        nib_base,
        axis="y",
        min_gap=0.020,
        name="finger lanes remain separated at rest",
    )
    with ctx.pose({object_model.get_articulation("round_middle_joint"): 0.78}):
        ctx.expect_gap(
            round_middle,
            round_base,
            axis="x",
            positive_elem="yoke_bridge",
            negative_elem="distal_plate_pos",
            min_gap=0.004,
            name="round middle yoke clears base side plate",
        )
    with ctx.pose({object_model.get_articulation("nib_middle_joint"): 0.70}):
        ctx.expect_gap(
            nib_middle,
            nib_base,
            axis="x",
            positive_elem="yoke_bridge",
            negative_elem="distal_plate_pos",
            min_gap=0.004,
            name="nib middle yoke clears base side plate",
        )

    round_pose = {
        object_model.get_articulation("round_base_joint"): 0.70,
        object_model.get_articulation("round_middle_joint"): 0.78,
        object_model.get_articulation("round_distal_joint"): 0.62,
    }
    nib_pose = {
        object_model.get_articulation("nib_base_joint"): 0.60,
        object_model.get_articulation("nib_middle_joint"): 0.70,
        object_model.get_articulation("nib_distal_joint"): 0.58,
    }
    round_rest = ctx.part_world_position(round_distal)
    nib_rest = ctx.part_world_position(nib_distal)
    with ctx.pose({**round_pose, **nib_pose}):
        ctx.expect_gap(
            round_base,
            nib_base,
            axis="y",
            min_gap=0.020,
            name="finger lanes remain separated when curled",
        )
        ctx.expect_gap(
            round_pedestal,
            round_base,
            axis="y",
            positive_elem="side_plate_pos",
            negative_elem="proximal_barrel",
            min_gap=0.002,
            max_gap=0.008,
            name="round base cheek clearance persists when curled",
        )
        round_curled = ctx.part_world_position(round_distal)
        nib_curled = ctx.part_world_position(nib_distal)
    ctx.check(
        "rounded chain curls upward",
        round_rest is not None
        and round_curled is not None
        and round_curled[2] > round_rest[2] + 0.045,
        details=f"rest={round_rest}, curled={round_curled}",
    )
    ctx.check(
        "flat-nib chain curls upward",
        nib_rest is not None
        and nib_curled is not None
        and nib_curled[2] > nib_rest[2] + 0.030,
        details=f"rest={nib_rest}, curled={nib_curled}",
    )

    return ctx.report()


object_model = build_object_model()
