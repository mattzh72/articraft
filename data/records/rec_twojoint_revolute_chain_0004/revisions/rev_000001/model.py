from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_joint_revolute_chain", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.34, 0.36, 0.40, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.55, 0.58, 0.62, 1.0))
    steel_light = model.material("steel_light", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.16, 0.17, 0.18, 1.0))

    shoulder_height = 0.08
    link1_length = 0.14
    link2_length = 0.11

    def y_cylinder(radius: float, length: float, xyz: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
        return (
            Cylinder(radius=radius, length=length),
            Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

    base = model.part("base_bracket")
    base.visual(
        Box((0.07, 0.03, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel_dark,
        name="base_plate",
    )
    base.visual(
        Box((0.05, 0.006, shoulder_height)),
        origin=Origin(xyz=(0.0, -0.003, shoulder_height / 2.0)),
        material=steel_dark,
        name="upright_web",
    )
    shoulder_boss_geom, shoulder_boss_origin = y_cylinder(
        radius=0.018,
        length=0.006,
        xyz=(0.0, -0.003, shoulder_height),
    )
    base.visual(
        shoulder_boss_geom,
        origin=shoulder_boss_origin,
        material=steel_light,
        name="shoulder_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.07, 0.03, shoulder_height)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, shoulder_height / 2.0)),
    )

    link1 = model.part("link_1")
    shoulder_hub_geom, shoulder_hub_origin = y_cylinder(
        radius=0.018,
        length=0.012,
        xyz=(0.0, 0.003, 0.0),
    )
    link1.visual(
        shoulder_hub_geom,
        origin=shoulder_hub_origin,
        material=steel_mid,
        name="shoulder_hub",
    )
    link1.visual(
        Box((0.018, 0.006, 0.104)),
        origin=Origin(xyz=(0.0, 0.003, 0.066)),
        material=steel_mid,
        name="arm_bar",
    )
    link1.visual(
        Box((0.018, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=steel_mid,
        name="cross_bridge",
    )
    link1.visual(
        Box((0.018, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.003, 0.124)),
        material=steel_mid,
        name="rear_connector",
    )
    elbow_boss_geom, elbow_boss_origin = y_cylinder(
        radius=0.014,
        length=0.010,
        xyz=(0.0, -0.001, link1_length),
    )
    link1.visual(
        elbow_boss_geom,
        origin=elbow_boss_origin,
        material=steel_light,
        name="elbow_boss",
    )
    link1.inertial = Inertial.from_geometry(
        Box((0.024, 0.014, link1_length)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, link1_length / 2.0)),
    )

    link2 = model.part("link_2")
    elbow_hub_geom, elbow_hub_origin = y_cylinder(
        radius=0.014,
        length=0.010,
        xyz=(0.0, 0.005, 0.0),
    )
    link2.visual(
        elbow_hub_geom,
        origin=elbow_hub_origin,
        material=steel_mid,
        name="elbow_hub",
    )
    link2.visual(
        Box((0.016, 0.006, link2_length)),
        origin=Origin(xyz=(0.0, 0.013, link2_length / 2.0)),
        material=steel_mid,
        name="forearm_bar",
    )
    link2.visual(
        Box((0.028, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.013, link2_length + 0.007)),
        material=rubber,
        name="tip_pad",
    )
    link2.inertial = Inertial.from_geometry(
        Box((0.028, 0.016, link2_length + 0.014)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.002, (link2_length + 0.014) / 2.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, shoulder_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(0.0, 0.0, link1_length)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_bracket")
    link1 = object_model.get_part("link_1")
    link2 = object_model.get_part("link_2")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

    shoulder_boss = base.get_visual("shoulder_boss")
    shoulder_hub = link1.get_visual("shoulder_hub")
    elbow_boss = link1.get_visual("elbow_boss")
    elbow_hub = link2.get_visual("elbow_hub")
    tip_pad = link2.get_visual("tip_pad")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        link1,
        base,
        elem_a=shoulder_hub,
        elem_b=shoulder_boss,
        reason="shoulder revolute uses a nested hinge barrel with slight seated interpenetration",
    )
    ctx.allow_overlap(
        link2,
        link1,
        elem_a=elbow_hub,
        elem_b=elbow_boss,
        reason="elbow revolute uses a nested hinge barrel with slight seated interpenetration",
    )

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

    ctx.expect_contact(
        link1,
        base,
        elem_a=shoulder_hub,
        elem_b=shoulder_boss,
        name="shoulder_hub_contacts_base_boss",
    )
    ctx.expect_overlap(
        link1,
        base,
        axes="xz",
        min_overlap=0.025,
        elem_a=shoulder_hub,
        elem_b=shoulder_boss,
        name="shoulder_joint_faces_align_in_xz",
    )
    ctx.expect_contact(
        link2,
        link1,
        elem_a=elbow_hub,
        elem_b=elbow_boss,
        name="elbow_hub_contacts_elbow_boss",
    )
    ctx.expect_overlap(
        link2,
        link1,
        axes="xz",
        min_overlap=0.02,
        elem_a=elbow_hub,
        elem_b=elbow_boss,
        name="elbow_joint_faces_align_in_xz",
    )
    def center_of_visual(part, visual):
        aabb = ctx.part_element_world_aabb(part, elem=visual)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    rest_pad_center = center_of_visual(link2, tip_pad)
    ctx.check(
        "rest_tip_pad_is_above_and_centered",
        rest_pad_center is not None
        and abs(rest_pad_center[0]) <= 0.002
        and abs(rest_pad_center[1] - 0.013) <= 0.003
        and 0.30 <= rest_pad_center[2] <= 0.34,
        details=f"tip_pad_center={rest_pad_center}",
    )

    with ctx.pose({shoulder_joint: math.pi / 2.0, elbow_joint: 0.0}):
        shoulder_swung_elbow = ctx.part_world_position(link2)
        ctx.expect_contact(link1, base, elem_a=shoulder_hub, elem_b=shoulder_boss)
        ctx.check(
            "positive_shoulder_pose_swings_elbow_to_positive_x",
            shoulder_swung_elbow is not None
            and 0.13 <= shoulder_swung_elbow[0] <= 0.15
            and abs(shoulder_swung_elbow[1]) <= 0.002
            and 0.07 <= shoulder_swung_elbow[2] <= 0.09,
            details=f"elbow_origin={shoulder_swung_elbow}",
        )

    with ctx.pose({shoulder_joint: -math.pi / 2.0, elbow_joint: 0.0}):
        shoulder_swung_elbow = ctx.part_world_position(link2)
        ctx.expect_contact(link1, base, elem_a=shoulder_hub, elem_b=shoulder_boss)
        ctx.check(
            "negative_shoulder_pose_swings_elbow_to_negative_x",
            shoulder_swung_elbow is not None
            and -0.15 <= shoulder_swung_elbow[0] <= -0.13
            and abs(shoulder_swung_elbow[1]) <= 0.002
            and 0.07 <= shoulder_swung_elbow[2] <= 0.09,
            details=f"elbow_origin={shoulder_swung_elbow}",
        )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: math.pi / 2.0}):
        elbow_swung_pad = center_of_visual(link2, tip_pad)
        ctx.expect_contact(link2, link1, elem_a=elbow_hub, elem_b=elbow_boss)
        ctx.check(
            "positive_elbow_pose_swings_pad_to_positive_x",
            elbow_swung_pad is not None
            and 0.10 <= elbow_swung_pad[0] <= 0.14
            and abs(elbow_swung_pad[1] - 0.013) <= 0.003
            and 0.21 <= elbow_swung_pad[2] <= 0.23,
            details=f"tip_pad_center={elbow_swung_pad}",
        )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: -math.pi / 2.0}):
        elbow_swung_pad = center_of_visual(link2, tip_pad)
        ctx.expect_contact(link2, link1, elem_a=elbow_hub, elem_b=elbow_boss)
        ctx.check(
            "negative_elbow_pose_swings_pad_to_negative_x",
            elbow_swung_pad is not None
            and -0.14 <= elbow_swung_pad[0] <= -0.10
            and abs(elbow_swung_pad[1] - 0.013) <= 0.003
            and 0.21 <= elbow_swung_pad[2] <= 0.23,
            details=f"tip_pad_center={elbow_swung_pad}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
