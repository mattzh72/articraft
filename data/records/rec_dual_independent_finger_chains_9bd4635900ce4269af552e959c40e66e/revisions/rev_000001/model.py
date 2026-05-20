from __future__ import annotations

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


JOINT_Z = 0.052
LINK_THICKNESS = 0.022
LINK_WIDTH = 0.028
BOSS_RADIUS = 0.020
PALM_BOSS_RADIUS = 0.030


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _add_segment(
    part,
    *,
    length: float,
    layer_z: float,
    material: Material,
    boss_material: Material,
    pad_material: Material | None = None,
    inward_y: float | None = None,
    is_tip: bool = False,
) -> None:
    """Add a flat digit link with round pin bosses at its joint centers."""
    part.visual(
        Box((length, LINK_WIDTH, LINK_THICKNESS)),
        origin=Origin(xyz=(length / 2.0, 0.0, layer_z)),
        material=material,
        name="link_plate",
    )
    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=LINK_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, layer_z)),
        material=boss_material,
        name="proximal_boss",
    )
    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=LINK_THICKNESS),
        origin=Origin(xyz=(length, 0.0, layer_z)),
        material=boss_material,
        name="distal_boss",
    )
    if is_tip and pad_material is not None and inward_y is not None:
        part.visual(
            Box((0.040, 0.014, 0.016)),
            origin=Origin(
                xyz=(
                    length + 0.010,
                    inward_y * (LINK_WIDTH / 2.0 + 0.006),
                    layer_z,
                )
            ),
            material=pad_material,
            name="fingertip_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_digit_service_gripper")

    body_mat = _mat("dark anodized body", (0.08, 0.09, 0.10, 1.0))
    cover_mat = _mat("blue service cover", (0.05, 0.16, 0.35, 1.0))
    link_mat = _mat("brushed steel links", (0.62, 0.66, 0.68, 1.0))
    boss_mat = _mat("polished pin bosses", (0.82, 0.84, 0.80, 1.0))
    rubber_mat = _mat("matte black rubber", (0.01, 0.01, 0.008, 1.0))
    fastener_mat = _mat("black oxide screws", (0.015, 0.015, 0.017, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((0.180, 0.150, 0.080)),
        origin=Origin(xyz=(0.000, 0.0, 0.040)),
        material=body_mat,
        name="dense_body",
    )
    palm.visual(
        Box((0.096, 0.112, 0.012)),
        origin=Origin(xyz=(-0.025, 0.0, 0.086)),
        material=cover_mat,
        name="service_cover",
    )
    palm.visual(
        Box((0.050, 0.118, 0.050)),
        origin=Origin(xyz=(0.075, 0.0, 0.039)),
        material=body_mat,
        name="front_shoulder",
    )

    # Separate clevis stacks support each digit directly; no cross-palm coupling
    # bar or shared linkage is present.
    base_x = 0.118
    finger_y = (0.050, -0.050)
    lower_ear_z = 0.0305
    upper_ear_z = 0.0735
    ear_thickness = 0.021
    for index, y in enumerate(finger_y):
        palm.visual(
            Cylinder(radius=PALM_BOSS_RADIUS, length=ear_thickness),
            origin=Origin(xyz=(base_x, y, lower_ear_z)),
            material=boss_mat,
            name=f"finger_{index}_lower_clevis",
        )
        palm.visual(
            Cylinder(radius=PALM_BOSS_RADIUS, length=ear_thickness),
            origin=Origin(xyz=(base_x, y, upper_ear_z)),
            material=boss_mat,
            name=f"finger_{index}_top_clevis",
        )

    for x in (-0.058, 0.008):
        for y in (-0.040, 0.040):
            palm.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(x, y, 0.082)),
                material=fastener_mat,
                name=f"cover_screw_{x:+.3f}_{y:+.3f}",
            )

    lengths = (0.105, 0.090, 0.070)
    layer_z = (0.0, LINK_THICKNESS, 0.0)
    for index, y in enumerate(finger_y):
        inward = -1.0 if y > 0.0 else 1.0
        proximal = model.part(f"finger_{index}_proximal")
        middle = model.part(f"finger_{index}_middle")
        tip = model.part(f"finger_{index}_tip")

        _add_segment(
            proximal,
            length=lengths[0],
            layer_z=layer_z[0],
            material=link_mat,
            boss_material=boss_mat,
        )
        _add_segment(
            middle,
            length=lengths[1],
            layer_z=layer_z[1],
            material=link_mat,
            boss_material=boss_mat,
        )
        _add_segment(
            tip,
            length=lengths[2],
            layer_z=layer_z[2],
            material=link_mat,
            boss_material=boss_mat,
            pad_material=rubber_mat,
            inward_y=inward,
            is_tip=True,
        )

        base_axis = (0.0, 0.0, -1.0 if y > 0.0 else 1.0)
        model.articulation(
            f"finger_{index}_base_joint",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(base_x, y, JOINT_Z)),
            axis=base_axis,
            motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=0.95),
        )
        model.articulation(
            f"finger_{index}_middle_joint",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(lengths[0], 0.0, 0.0)),
            axis=base_axis,
            motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=0.0, upper=1.10),
        )
        model.articulation(
            f"finger_{index}_tip_joint",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=tip,
            origin=Origin(xyz=(lengths[1], 0.0, 0.0)),
            axis=base_axis,
            motion_limits=MotionLimits(effort=8.0, velocity=3.5, lower=0.0, upper=0.85),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joint_names = [joint.name for joint in object_model.articulations]
    expected_joints = [
        f"finger_{index}_{joint}"
        for index in range(2)
        for joint in ("base_joint", "middle_joint", "tip_joint")
    ]
    ctx.check(
        "two independent three-joint digit chains",
        sorted(joint_names) == sorted(expected_joints)
        and all(
            object_model.get_articulation(name).articulation_type
            == ArticulationType.REVOLUTE
            for name in expected_joints
        )
        and all(object_model.get_articulation(name).mimic is None for name in expected_joints),
        details=f"joints={joint_names}",
    )

    palm = object_model.get_part("palm")
    for index in range(2):
        proximal = object_model.get_part(f"finger_{index}_proximal")
        middle = object_model.get_part(f"finger_{index}_middle")
        tip = object_model.get_part(f"finger_{index}_tip")
        ctx.expect_contact(
            palm,
            proximal,
            elem_a=f"finger_{index}_top_clevis",
            elem_b="proximal_boss",
            contact_tol=0.0005,
            name=f"finger {index} base pin boss is captured by palm clevis",
        )
        ctx.expect_contact(
            proximal,
            middle,
            elem_a="distal_boss",
            elem_b="proximal_boss",
            contact_tol=0.0005,
            name=f"finger {index} middle joint bosses meet",
        )
        ctx.expect_contact(
            middle,
            tip,
            elem_a="distal_boss",
            elem_b="proximal_boss",
            contact_tol=0.0005,
            name=f"finger {index} tip joint bosses meet",
        )

    rest_tip_0 = ctx.part_world_aabb("finger_0_tip")
    rest_tip_1 = ctx.part_world_aabb("finger_1_tip")
    with ctx.pose(
        {
            "finger_0_base_joint": 0.35,
            "finger_0_middle_joint": 0.35,
            "finger_0_tip_joint": 0.25,
            "finger_1_base_joint": 0.35,
            "finger_1_middle_joint": 0.35,
            "finger_1_tip_joint": 0.25,
        }
    ):
        curled_tip_0 = ctx.part_world_aabb("finger_0_tip")
        curled_tip_1 = ctx.part_world_aabb("finger_1_tip")

    def _center_y(aabb):
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) / 2.0

    ctx.check(
        "both digit chains curl toward the grasp gap",
        rest_tip_0 is not None
        and rest_tip_1 is not None
        and curled_tip_0 is not None
        and curled_tip_1 is not None
        and _center_y(curled_tip_0) < _center_y(rest_tip_0) - 0.015
        and _center_y(curled_tip_1) > _center_y(rest_tip_1) + 0.015,
        details=(
            f"rest=({_center_y(rest_tip_0)}, {_center_y(rest_tip_1)}), "
            f"curled=({_center_y(curled_tip_0)}, {_center_y(curled_tip_1)})"
        ),
    )

    return ctx.report()


object_model = build_object_model()
