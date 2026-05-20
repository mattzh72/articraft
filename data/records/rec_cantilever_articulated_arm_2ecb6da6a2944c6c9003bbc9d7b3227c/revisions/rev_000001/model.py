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


ROOT_PIVOT_Z = 0.200
UPPER_LENGTH = 0.420
FORE_LENGTH = 0.320


def _add_y_cylinder(part, name: str, radius: float, length: float, xyz, material: str) -> None:
    """Add a cylinder whose axis is the local Y axis."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_x_cylinder(part, name: str, radius: float, length: float, xyz, material: str) -> None:
    """Add a cylinder whose axis is the local X axis."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_box_tube(part, prefix: str, *, x0: float, x1: float, width: float, height: float, wall: float, material: str) -> None:
    """Four welded plates forming a rectangular box-section beam open at the ends."""
    length = x1 - x0
    xc = (x0 + x1) / 2.0
    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(xc, 0.0, height / 2.0 - wall / 2.0)),
        material=material,
        name=f"{prefix}_top_wall",
    )
    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(xc, 0.0, -height / 2.0 + wall / 2.0)),
        material=material,
        name=f"{prefix}_bottom_wall",
    )
    part.visual(
        Box((length, wall, height)),
        origin=Origin(xyz=(xc, width / 2.0 - wall / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_side_wall_pos",
    )
    part.visual(
        Box((length, wall, height)),
        origin=Origin(xyz=(xc, -width / 2.0 + wall / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_side_wall_neg",
    )


def _add_clevis(part, prefix: str, *, pivot_x: float, pivot_z: float, material: str, cap_material: str) -> None:
    """Industrial fork around a local Y-axis pin, open at the pivot side."""
    part.visual(
        Box((0.120, 0.014, 0.090)),
        origin=Origin(xyz=(pivot_x, 0.047, pivot_z)),
        material=material,
        name=f"{prefix}_cheek_pos",
    )
    part.visual(
        Box((0.120, 0.014, 0.090)),
        origin=Origin(xyz=(pivot_x, -0.047, pivot_z)),
        material=material,
        name=f"{prefix}_cheek_neg",
    )
    # A rear bridge visibly ties the two cheeks back into the carrying link.
    part.visual(
        Box((0.040, 0.108, 0.076)),
        origin=Origin(xyz=(pivot_x - 0.065, 0.0, pivot_z)),
        material=material,
        name=f"{prefix}_rear_bridge",
    )
    # Exposed outer pin bosses; they do not pass through the child lug proxy.
    _add_y_cylinder(part, f"{prefix}_pin_cap_pos", 0.020, 0.010, (pivot_x, 0.059, pivot_z), cap_material)
    _add_y_cylinder(part, f"{prefix}_pin_cap_neg", 0.020, 0.010, (pivot_x, -0.059, pivot_z), cap_material)
    _add_y_cylinder(part, f"{prefix}_bore_pos", 0.008, 0.003, (pivot_x, 0.065, pivot_z), "dark_bore")
    _add_y_cylinder(part, f"{prefix}_bore_neg", 0.008, 0.003, (pivot_x, -0.065, pivot_z), "dark_bore")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_industrial_cantilever_arm")

    model.material("root_powdercoat", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("link_orange", rgba=(0.92, 0.38, 0.08, 1.0))
    model.material("end_plate_black", rgba=(0.06, 0.065, 0.07, 1.0))
    model.material("pin_zinc", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("dark_bore", rgba=(0.01, 0.012, 0.014, 1.0))
    model.material("wear_strip", rgba=(0.11, 0.12, 0.13, 1.0))

    root = model.part("root_block")
    # Floor-mounted reinforced base and upright root block.
    root.visual(
        Box((0.300, 0.190, 0.026)),
        origin=Origin(xyz=(-0.060, 0.0, 0.013)),
        material="root_powdercoat",
        name="base_plate",
    )
    root.visual(
        Box((0.115, 0.135, 0.205)),
        origin=Origin(xyz=(-0.110, 0.0, 0.128)),
        material="root_powdercoat",
        name="root_upright",
    )
    root.visual(
        Box((0.150, 0.014, 0.026)),
        origin=Origin(xyz=(-0.100, 0.075, 0.085), rpy=(0.0, -0.58, 0.0)),
        material="root_powdercoat",
        name="side_gusset_pos",
    )
    root.visual(
        Box((0.150, 0.014, 0.026)),
        origin=Origin(xyz=(-0.100, -0.075, 0.085), rpy=(0.0, -0.58, 0.0)),
        material="root_powdercoat",
        name="side_gusset_neg",
    )
    root.visual(
        Box((0.030, 0.108, 0.116)),
        origin=Origin(xyz=(-0.052, 0.0, ROOT_PIVOT_Z)),
        material="root_powdercoat",
        name="root_clevis_backbone",
    )
    root.visual(
        Box((0.108, 0.108, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, ROOT_PIVOT_Z - 0.062)),
        material="root_powdercoat",
        name="root_clevis_saddle",
    )
    root.visual(
        Box((0.100, 0.014, 0.110)),
        origin=Origin(xyz=(0.005, 0.047, ROOT_PIVOT_Z)),
        material="root_powdercoat",
        name="root_clevis_cheek_pos",
    )
    root.visual(
        Box((0.100, 0.014, 0.110)),
        origin=Origin(xyz=(0.005, -0.047, ROOT_PIVOT_Z)),
        material="root_powdercoat",
        name="root_clevis_cheek_neg",
    )
    _add_y_cylinder(root, "root_pin_shaft", 0.014, 0.118, (0.0, 0.0, ROOT_PIVOT_Z), "pin_zinc")
    _add_y_cylinder(root, "root_pin_cap_pos", 0.021, 0.010, (0.0, 0.059, ROOT_PIVOT_Z), "pin_zinc")
    _add_y_cylinder(root, "root_pin_cap_neg", 0.021, 0.010, (0.0, -0.059, ROOT_PIVOT_Z), "pin_zinc")
    for x in (-0.165, 0.045):
        for y in (-0.065, 0.065):
            _add_y_cylinder(root, f"anchor_bolt_{x:+.3f}_{y:+.3f}", 0.010, 0.007, (x, y, 0.030), "pin_zinc")

    upper = model.part("upper_link")
    _add_y_cylinder(upper, "shoulder_lug", 0.026, 0.048, (0.0, 0.0, 0.0), "pin_zinc")
    upper.visual(
        Box((0.050, 0.046, 0.046)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material="link_orange",
        name="shoulder_neck",
    )
    _add_box_tube(upper, "upper_box", x0=0.055, x1=0.345, width=0.052, height=0.066, wall=0.010, material="link_orange")
    upper.visual(
        Box((0.230, 0.004, 0.030)),
        origin=Origin(xyz=(0.200, 0.027, 0.0)),
        material="wear_strip",
        name="upper_side_inset_pos",
    )
    upper.visual(
        Box((0.230, 0.004, 0.030)),
        origin=Origin(xyz=(0.200, -0.027, 0.0)),
        material="wear_strip",
        name="upper_side_inset_neg",
    )
    _add_clevis(upper, "elbow", pivot_x=UPPER_LENGTH, pivot_z=0.0, material="link_orange", cap_material="pin_zinc")
    _add_y_cylinder(upper, "elbow_pin_shaft", 0.013, 0.116, (UPPER_LENGTH, 0.0, 0.0), "pin_zinc")

    fore = model.part("forelink")
    _add_y_cylinder(fore, "elbow_lug", 0.024, 0.046, (0.0, 0.0, 0.0), "pin_zinc")
    fore.visual(
        Box((0.048, 0.044, 0.044)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material="link_orange",
        name="elbow_neck",
    )
    _add_box_tube(fore, "fore_box", x0=0.052, x1=0.245, width=0.050, height=0.060, wall=0.009, material="link_orange")
    fore.visual(
        Box((0.145, 0.004, 0.026)),
        origin=Origin(xyz=(0.150, 0.026, 0.0)),
        material="wear_strip",
        name="fore_side_inset_pos",
    )
    fore.visual(
        Box((0.145, 0.004, 0.026)),
        origin=Origin(xyz=(0.150, -0.026, 0.0)),
        material="wear_strip",
        name="fore_side_inset_neg",
    )
    _add_clevis(fore, "wrist", pivot_x=FORE_LENGTH, pivot_z=0.0, material="link_orange", cap_material="pin_zinc")
    _add_y_cylinder(fore, "wrist_pin_shaft", 0.012, 0.114, (FORE_LENGTH, 0.0, 0.0), "pin_zinc")

    end = model.part("end_plate")
    _add_y_cylinder(end, "wrist_lug", 0.022, 0.044, (0.0, 0.0, 0.0), "pin_zinc")
    end.visual(
        Box((0.070, 0.048, 0.050)),
        origin=Origin(xyz=(0.049, 0.0, 0.0)),
        material="end_plate_black",
        name="wrist_neck",
    )
    end.visual(
        Box((0.028, 0.120, 0.120)),
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        material="end_plate_black",
        name="tool_plate",
    )
    for y in (-0.040, 0.040):
        for z in (-0.040, 0.040):
            _add_x_cylinder(end, f"tool_bolt_{y:+.2f}_{z:+.2f}", 0.006, 0.008, (0.107, y, z), "pin_zinc")

    model.articulation(
        "root_hinge",
        ArticulationType.REVOLUTE,
        parent=root,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, ROOT_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.30, upper=1.20, effort=120.0, velocity=1.0),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=fore,
        origin=Origin(xyz=(UPPER_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.25, upper=1.25, effort=90.0, velocity=1.2),
    )
    model.articulation(
        "wrist_hinge",
        ArticulationType.REVOLUTE,
        parent=fore,
        child=end,
        origin=Origin(xyz=(FORE_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.45, upper=1.45, effort=45.0, velocity=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_block")
    upper = object_model.get_part("upper_link")
    fore = object_model.get_part("forelink")
    end = object_model.get_part("end_plate")
    root_hinge = object_model.get_articulation("root_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    wrist_hinge = object_model.get_articulation("wrist_hinge")

    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    ctx.allow_overlap(
        root,
        upper,
        elem_a="root_pin_shaft",
        elem_b="shoulder_lug",
        reason="The shoulder pin is intentionally captured through the lug bore proxy.",
    )
    ctx.allow_overlap(
        upper,
        fore,
        elem_a="elbow_pin_shaft",
        elem_b="elbow_lug",
        reason="The elbow pin is intentionally captured through the forelink lug bore proxy.",
    )
    ctx.allow_overlap(
        fore,
        end,
        elem_a="wrist_pin_shaft",
        elem_b="wrist_lug",
        reason="The wrist pin is intentionally captured through the end-plate lug bore proxy.",
    )

    ctx.expect_overlap(
        root,
        upper,
        axes="xyz",
        elem_a="root_pin_shaft",
        elem_b="shoulder_lug",
        min_overlap=0.012,
        name="shoulder pin passes through lug",
    )
    ctx.expect_overlap(
        upper,
        fore,
        axes="xyz",
        elem_a="elbow_pin_shaft",
        elem_b="elbow_lug",
        min_overlap=0.012,
        name="elbow pin passes through lug",
    )
    ctx.expect_overlap(
        fore,
        end,
        axes="xyz",
        elem_a="wrist_pin_shaft",
        elem_b="wrist_lug",
        min_overlap=0.011,
        name="wrist pin passes through lug",
    )

    # Each child lug sits between two clevis cheeks with real side clearance.
    ctx.expect_gap(
        root,
        upper,
        axis="y",
        positive_elem="root_clevis_cheek_pos",
        negative_elem="shoulder_lug",
        min_gap=0.010,
        max_gap=0.022,
        name="shoulder lug clears positive root clevis cheek",
    )
    ctx.expect_gap(
        upper,
        root,
        axis="y",
        positive_elem="shoulder_lug",
        negative_elem="root_clevis_cheek_neg",
        min_gap=0.010,
        max_gap=0.022,
        name="shoulder lug clears negative root clevis cheek",
    )
    ctx.expect_gap(
        upper,
        fore,
        axis="y",
        positive_elem="elbow_cheek_pos",
        negative_elem="elbow_lug",
        min_gap=0.010,
        max_gap=0.024,
        name="elbow lug clears positive upper clevis cheek",
    )
    ctx.expect_gap(
        fore,
        upper,
        axis="y",
        positive_elem="elbow_lug",
        negative_elem="elbow_cheek_neg",
        min_gap=0.010,
        max_gap=0.024,
        name="elbow lug clears negative upper clevis cheek",
    )
    ctx.expect_gap(
        fore,
        end,
        axis="y",
        positive_elem="wrist_cheek_pos",
        negative_elem="wrist_lug",
        min_gap=0.010,
        max_gap=0.024,
        name="wrist lug clears positive forelink clevis cheek",
    )
    ctx.expect_gap(
        end,
        fore,
        axis="y",
        positive_elem="wrist_lug",
        negative_elem="wrist_cheek_neg",
        min_gap=0.010,
        max_gap=0.024,
        name="wrist lug clears negative forelink clevis cheek",
    )

    ctx.expect_overlap(
        upper,
        root,
        axes="xz",
        elem_a="shoulder_lug",
        elem_b="root_clevis_cheek_pos",
        min_overlap=0.035,
        name="shoulder lug is centered in root clevis span",
    )
    ctx.expect_overlap(
        fore,
        upper,
        axes="xz",
        elem_a="elbow_lug",
        elem_b="elbow_cheek_pos",
        min_overlap=0.032,
        name="elbow lug is centered in upper clevis span",
    )
    ctx.expect_overlap(
        end,
        fore,
        axes="xz",
        elem_a="wrist_lug",
        elem_b="wrist_cheek_pos",
        min_overlap=0.030,
        name="wrist lug is centered in forelink clevis span",
    )

    rest_elbow = ctx.part_world_position(fore)
    with ctx.pose({root_hinge: 0.55}):
        raised_elbow = ctx.part_world_position(fore)
    ctx.check(
        "root hinge pitches upper link upward",
        rest_elbow is not None and raised_elbow is not None and raised_elbow[2] > rest_elbow[2] + 0.18,
        details=f"rest={rest_elbow}, raised={raised_elbow}",
    )

    rest_wrist = ctx.part_world_position(end)
    with ctx.pose({elbow_hinge: 0.55}):
        raised_wrist = ctx.part_world_position(end)
    ctx.check(
        "elbow hinge pitches forelink upward",
        rest_wrist is not None and raised_wrist is not None and raised_wrist[2] > rest_wrist[2] + 0.13,
        details=f"rest={rest_wrist}, raised={raised_wrist}",
    )

    rest_aabb = ctx.part_world_aabb(end)
    with ctx.pose({wrist_hinge: 0.80}):
        raised_aabb = ctx.part_world_aabb(end)
    ctx.check(
        "wrist hinge tilts tool plate",
        rest_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > rest_aabb[1][2] + 0.030,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
