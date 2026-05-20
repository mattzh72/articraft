from __future__ import annotations

from math import isclose, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


PLATE_SIZE = (0.26, 0.16, 0.012)
JOINT_Z = 0.035
LINK_THICKNESS = 0.008
LINK_WIDTH = 0.040
LOBE_RADIUS = 0.027
PIN_RADIUS = 0.008
HOLE_RADIUS = PIN_RADIUS
PIN_LENGTH = 0.038
LOW_LAYER_Z = -0.006
HIGH_LAYER_Z = 0.006
LINK_LENGTHS = (0.150, 0.135, 0.120)
END_TAB_LENGTH = 0.095


def _cut_cylinder_at(shape: cq.Workplane, x: float, y: float, radius: float, height: float) -> cq.Workplane:
    cutter = (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, -0.002))
    )
    return shape.cut(cutter)


def _link_shape(length: float, layer_z: float) -> cq.Workplane:
    """Flat link in a pivot-centered local frame, with a real proximal pin hole."""
    body = (
        cq.Workplane("XY")
        .center(length / 2.0, 0.0)
        .rect(length, LINK_WIDTH)
        .extrude(LINK_THICKNESS)
    )
    proximal_lobe = cq.Workplane("XY").circle(LOBE_RADIUS).extrude(LINK_THICKNESS)
    distal_lobe = (
        cq.Workplane("XY")
        .center(length, 0.0)
        .circle(LOBE_RADIUS)
        .extrude(LINK_THICKNESS)
    )
    shape = body.union(proximal_lobe).union(distal_lobe)
    shape = _cut_cylinder_at(shape, 0.0, 0.0, HOLE_RADIUS, LINK_THICKNESS + 0.004)
    return shape.translate((0.0, 0.0, layer_z - LINK_THICKNESS / 2.0))


def _end_tab_shape(layer_z: float) -> cq.Workplane:
    """Compact final tab with a pivot eye and a short rounded working end."""
    body = (
        cq.Workplane("XY")
        .center(END_TAB_LENGTH / 2.0, 0.0)
        .rect(END_TAB_LENGTH, 0.034)
        .extrude(LINK_THICKNESS)
    )
    pivot_eye = cq.Workplane("XY").circle(0.024).extrude(LINK_THICKNESS)
    rounded_tip = (
        cq.Workplane("XY")
        .center(END_TAB_LENGTH, 0.0)
        .circle(0.018)
        .extrude(LINK_THICKNESS)
    )
    shape = body.union(pivot_eye).union(rounded_tip)
    shape = _cut_cylinder_at(shape, 0.0, 0.0, HOLE_RADIUS, LINK_THICKNESS + 0.004)
    return shape.translate((0.0, 0.0, layer_z - LINK_THICKNESS / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_four_joint_chain")

    model.material("blackened_steel", rgba=(0.035, 0.037, 0.040, 1.0))
    model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("dark_pin", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("blue_anodized", rgba=(0.12, 0.25, 0.42, 1.0))
    model.material("graphite_anodized", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("rubber_black", rgba=(0.005, 0.005, 0.004, 1.0))

    mount = model.part("mount")
    mount.visual(
        Box(PLATE_SIZE),
        origin=Origin(xyz=(-0.070, 0.0, PLATE_SIZE[2] / 2.0)),
        material="blackened_steel",
        name="plate",
    )
    mount.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material="brushed_steel",
        name="root_boss",
    )
    mount.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        material="dark_pin",
        name="root_pin",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((-0.150, -0.055), (-0.150, 0.055), (0.005, -0.055), (0.005, 0.055))
    ):
        mount.visual(
            Cylinder(radius=0.011, length=0.003),
            origin=Origin(xyz=(x_pos, y_pos, PLATE_SIZE[2] + 0.0015)),
            material="brushed_steel",
            name=f"screw_{index}",
        )
        mount.visual(
            Box((0.018, 0.003, 0.001)),
            origin=Origin(xyz=(x_pos, y_pos, PLATE_SIZE[2] + 0.0031), rpy=(0.0, 0.0, pi / 4.0)),
            material="dark_pin",
            name=f"screw_slot_{index}",
        )

    link_0 = model.part("link_0")
    link_0.visual(
        mesh_from_cadquery(_link_shape(LINK_LENGTHS[0], HIGH_LAYER_Z), "link_0_body"),
        material="blue_anodized",
        name="body",
    )
    link_0.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, 0.0)),
        material="dark_pin",
        name="distal_pin",
    )
    link_0.visual(
        Box((LINK_LENGTHS[0] * 0.55, 0.006, 0.0012)),
        origin=Origin(xyz=(LINK_LENGTHS[0] * 0.52, 0.0, HIGH_LAYER_Z + LINK_THICKNESS / 2.0 + 0.0006)),
        material="graphite_anodized",
        name="top_groove",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_link_shape(LINK_LENGTHS[1], LOW_LAYER_Z), "link_1_body"),
        material="graphite_anodized",
        name="body",
    )
    link_1.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        material="dark_pin",
        name="distal_pin",
    )
    link_1.visual(
        Box((LINK_LENGTHS[1] * 0.54, 0.006, 0.0012)),
        origin=Origin(xyz=(LINK_LENGTHS[1] * 0.52, 0.0, LOW_LAYER_Z + LINK_THICKNESS / 2.0 + 0.0006)),
        material="blackened_steel",
        name="top_groove",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_link_shape(LINK_LENGTHS[2], HIGH_LAYER_Z), "link_2_body"),
        material="blue_anodized",
        name="body",
    )
    link_2.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(LINK_LENGTHS[2], 0.0, 0.0)),
        material="dark_pin",
        name="distal_pin",
    )
    link_2.visual(
        Box((LINK_LENGTHS[2] * 0.52, 0.006, 0.0012)),
        origin=Origin(xyz=(LINK_LENGTHS[2] * 0.52, 0.0, HIGH_LAYER_Z + LINK_THICKNESS / 2.0 + 0.0006)),
        material="graphite_anodized",
        name="top_groove",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_end_tab_shape(LOW_LAYER_Z), "end_tab_body"),
        material="graphite_anodized",
        name="body",
    )
    end_tab.visual(
        Box((0.028, 0.026, 0.003)),
        origin=Origin(xyz=(END_TAB_LENGTH * 0.82, 0.0, LOW_LAYER_Z + LINK_THICKNESS / 2.0 + 0.0015)),
        material="rubber_black",
        name="end_pad",
    )

    limits = MotionLimits(effort=4.0, velocity=2.0, lower=-1.20, upper=1.20)
    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "joint_0",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "joint_1",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "joint_2",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_tab,
        origin=Origin(xyz=(LINK_LENGTHS[2], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    joints = [object_model.get_articulation(name) for name in ("root_pivot", "joint_0", "joint_1", "joint_2")]
    ctx.check(
        "four revolute joints",
        len(joints) == 4 and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints),
        details=f"joints={[joint.name for joint in joints]}",
    )
    ctx.check(
        "all axes bend in horizontal plane",
        all(tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0) for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    mount = object_model.get_part("mount")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_tab = object_model.get_part("end_tab")

    ctx.allow_overlap(
        mount,
        link_0,
        elem_a="root_pin",
        elem_b="body",
        reason="The root pivot pin is intentionally modeled as a tight captured shaft inside the link's bearing eye.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="body",
        reason="The first distal pin is intentionally captured inside the next link's bearing eye.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        elem_a="distal_pin",
        elem_b="body",
        reason="The second distal pin is intentionally captured inside the next link's bearing eye.",
    )
    ctx.allow_overlap(
        link_2,
        end_tab,
        elem_a="distal_pin",
        elem_b="body",
        reason="The final distal pin is intentionally captured inside the compact end tab eye.",
    )

    ctx.expect_gap(
        link_0,
        mount,
        axis="z",
        positive_elem="body",
        negative_elem="root_boss",
        min_gap=0.002,
        name="first link sits above base boss",
    )
    ctx.expect_overlap(
        link_0,
        mount,
        axes="xy",
        elem_a="body",
        elem_b="root_pin",
        min_overlap=0.012,
        name="first link eye surrounds root pin",
    )
    ctx.expect_overlap(
        link_1,
        link_0,
        axes="xy",
        elem_a="body",
        elem_b="distal_pin",
        min_overlap=0.012,
        name="second link eye surrounds first distal pin",
    )
    ctx.expect_overlap(
        link_2,
        link_1,
        axes="xy",
        elem_a="body",
        elem_b="distal_pin",
        min_overlap=0.012,
        name="third link eye surrounds second distal pin",
    )
    ctx.expect_overlap(
        end_tab,
        link_2,
        axes="xy",
        elem_a="body",
        elem_b="distal_pin",
        min_overlap=0.012,
        name="end tab eye surrounds final pin",
    )

    rest_tip = ctx.part_world_position(end_tab)
    with ctx.pose({"root_pivot": 0.35, "joint_0": -0.45, "joint_1": 0.55, "joint_2": -0.30}):
        bent_tip = ctx.part_world_position(end_tab)
    ctx.check(
        "chain pose bends in plane",
        rest_tip is not None
        and bent_tip is not None
        and abs(bent_tip[1] - rest_tip[1]) > 0.020
        and isclose(bent_tip[2], rest_tip[2], abs_tol=0.001),
        details=f"rest={rest_tip}, bent={bent_tip}",
    )

    return ctx.report()


object_model = build_object_model()
