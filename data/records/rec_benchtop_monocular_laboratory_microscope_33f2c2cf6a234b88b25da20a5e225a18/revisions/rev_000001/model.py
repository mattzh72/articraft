from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cylinder_between_yz(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Return a cylinder transform for struts that lie in a constant-X YZ plane."""
    x0, y0, z0 = start
    x1, y1, z1 = end
    dy = y1 - y0
    dz = z1 - z0
    length = math.sqrt(dy * dy + dz * dz)
    # A URDF cylinder runs along local +Z.  Rotating about X maps local Z to
    # (0, -sin(roll), cos(roll)), so this aligns it to the YZ vector.
    roll = -math.atan2(dy, dz)
    return Origin(xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5), rpy=(roll, 0.0, 0.0)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_monocular_microscope")

    enamel = Material("warm_gray_enamel", rgba=(0.58, 0.60, 0.56, 1.0))
    dark = Material("blackened_metal", rgba=(0.015, 0.017, 0.018, 1.0))
    satin = Material("brushed_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    stage_black = Material("matte_stage_black", rgba=(0.02, 0.022, 0.022, 1.0))
    glass = Material("greenish_glass", rgba=(0.38, 0.68, 0.58, 0.65))
    brass = Material("warm_brass", rgba=(0.86, 0.63, 0.25, 1.0))
    white = Material("lamp_diffuser", rgba=(1.0, 0.92, 0.62, 0.85))

    stand = model.part("stand")

    base_shape = (
        cq.Workplane("XY")
        .box(0.220, 0.285, 0.030)
        .edges("|Z")
        .fillet(0.020)
        .edges(">Z")
        .fillet(0.006)
    )
    stand.visual(
        mesh_from_cadquery(base_shape, "rounded_base", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark,
        name="rounded_base",
    )
    stand.visual(
        Box((0.075, 0.085, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=enamel,
        name="illuminator_riser",
    )

    lamp_shape = (
        cq.Workplane("XY")
        .box(0.108, 0.112, 0.070)
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.004)
    )
    stand.visual(
        mesh_from_cadquery(lamp_shape, "lamp_housing", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=enamel,
        name="lamp_housing",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.1725)),
        material=white,
        name="frosted_lamp_window",
    )

    # The fore-aft guide is a fixed lower slideway below the square stage.
    stand.visual(
        Box((0.118, 0.182, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=satin,
        name="guide_bed",
    )
    stand.visual(
        Box((0.012, 0.180, 0.012)),
        origin=Origin(xyz=(-0.047, 0.0, 0.188)),
        material=satin,
        name="guide_rail_0",
    )
    stand.visual(
        Box((0.012, 0.180, 0.012)),
        origin=Origin(xyz=(0.047, 0.0, 0.188)),
        material=satin,
        name="guide_rail_1",
    )

    # Rear yoke post and rounded struts: the microscope reads as a cast yoke arm
    # rather than a simple vertical column.
    stand.visual(
        Cylinder(radius=0.019, length=0.295),
        origin=Origin(xyz=(0.0, -0.094, 0.169), rpy=(0.0, 0.0, 0.0)),
        material=enamel,
        name="yoke_post",
    )
    for name, start, end, radius in (
        ("upper_yoke_arm", (0.0, -0.094, 0.305), (0.0, -0.015, 0.360), 0.015),
        ("lower_yoke_arm", (0.0, -0.094, 0.170), (0.0, -0.084, 0.222), 0.010),
    ):
        origin, length = _cylinder_between_yz(start, end)
        stand.visual(
            Cylinder(radius=radius, length=length),
            origin=origin,
            material=enamel,
            name=name,
        )

    stand.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        material=satin,
        name="tube_lower_collar",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        material=dark,
        name="monocular_tube",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.445)),
        material=satin,
        name="tube_top_collar",
    )
    stand.visual(
        Cylinder(radius=0.016, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        material=dark,
        name="eyepiece",
    )
    stand.visual(
        Cylinder(radius=0.011, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.509)),
        material=glass,
        name="eyepiece_lens",
    )

    # Fixed focus shaft for the side focusing knob.
    stand.visual(
        Cylinder(radius=0.007, length=0.060),
        origin=Origin(xyz=(0.048, -0.055, 0.265), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="focus_shaft",
    )
    stand.visual(
        Box((0.030, 0.050, 0.050)),
        origin=Origin(xyz=(0.018, -0.074, 0.265)),
        material=enamel,
        name="focus_gear_box",
    )

    # Fixed hinge knuckles for the lamp-access door.  The moving door carries the
    # center knuckle and panel.
    for name, zc in (("door_hinge_knuckle_0", 0.1155), ("door_hinge_knuckle_1", 0.1555)):
        stand.visual(
            Cylinder(radius=0.0042, length=0.013),
            origin=Origin(xyz=(-0.054, 0.059, zc)),
            material=satin,
            name=name,
        )
    stand.visual(
        Box((0.006, 0.010, 0.058)),
        origin=Origin(xyz=(-0.0565, 0.054, 0.135)),
        material=satin,
        name="door_hinge_leaf",
    )

    stage = model.part("stage")
    stage.visual(
        Box((0.138, 0.138, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stage_black,
        name="square_stage_plate",
    )
    stage.visual(
        Box((0.090, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=satin,
        name="stage_carriage",
    )
    stage.visual(
        Box((0.042, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.050, 0.006)),
        material=satin,
        name="front_slide_scale",
    )
    for x in (-0.052, 0.052):
        stage.visual(
            Box((0.010, 0.040, 0.004)),
            origin=Origin(xyz=(x, 0.030, 0.0055)),
            material=satin,
            name=f"stage_clip_{0 if x < 0 else 1}",
        )

    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, 0.216)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.08, lower=-0.035, upper=0.035),
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=satin,
        name="turret_disk",
    )
    nosepiece.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=dark,
        name="turret_boss",
    )
    objective_specs = (
        ("objective_0", 0.023, 0.000, 0.0060, 0.044, -0.034, brass),
        ("objective_1", -0.0115, 0.020, 0.0052, 0.038, -0.031, satin),
        ("objective_2", -0.0115, -0.020, 0.0048, 0.032, -0.028, satin),
    )
    for name, x, y, radius, length, zc, mat in objective_specs:
        nosepiece.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, zc)),
            material=mat,
            name=name,
        )
        nosepiece.visual(
            Cylinder(radius=radius * 0.72, length=0.003),
            origin=Origin(xyz=(x, y, zc - length * 0.5 - 0.0015)),
            material=glass,
            name=f"{name}_lens",
        )

    model.articulation(
        "nosepiece_turret",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=nosepiece,
        origin=Origin(xyz=(0.0, 0.0, 0.282)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    lamp_door = model.part("lamp_door")
    lamp_door.visual(
        Box((0.072, 0.004, 0.045)),
        origin=Origin(xyz=(0.036, 0.0025, 0.0)),
        material=dark,
        name="door_panel",
    )
    lamp_door.visual(
        Cylinder(radius=0.0040, length=0.024),
        origin=Origin(xyz=(0.0, 0.0025, 0.0)),
        material=satin,
        name="door_hinge_knuckle",
    )
    lamp_door.visual(
        Cylinder(radius=0.003, length=0.003),
        origin=Origin(xyz=(0.064, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="door_pull",
    )
    model.articulation(
        "lamp_door_hinge",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=lamp_door,
        origin=Origin(xyz=(-0.054, 0.057, 0.135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=0.0, upper=1.35),
    )

    side_focus = model.part("side_focus")
    focus_knob = KnobGeometry(
        0.042,
        0.024,
        body_style="cylindrical",
        edge_radius=0.001,
        grip=KnobGrip(style="ribbed", count=24, depth=0.0012, width=0.0020),
    )
    side_focus.visual(
        mesh_from_geometry(focus_knob, "side_focus_knob"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="focus_knob",
    )
    model.articulation(
        "focus_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=side_focus,
        origin=Origin(xyz=(0.078, -0.055, 0.265)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    stage = object_model.get_part("stage")
    nosepiece = object_model.get_part("nosepiece")
    lamp_door = object_model.get_part("lamp_door")
    side_focus = object_model.get_part("side_focus")
    stage_slide = object_model.get_articulation("stage_slide")
    turret = object_model.get_articulation("nosepiece_turret")
    door_hinge = object_model.get_articulation("lamp_door_hinge")

    def aabb_center(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.allow_overlap(
        side_focus,
        stand,
        elem_a="focus_knob",
        elem_b="focus_shaft",
        reason="The focusing knob is intentionally seated over the fixed shaft so the control reads mechanically captured.",
    )
    ctx.expect_overlap(
        side_focus,
        stand,
        axes="x",
        elem_a="focus_knob",
        elem_b="focus_shaft",
        min_overlap=0.010,
        name="focus knob is retained on shaft",
    )

    # The square stage rides on the lower guide and translates in fore-aft Y.
    ctx.expect_gap(
        stage,
        stand,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0005,
        positive_elem="stage_carriage",
        negative_elem="guide_rail_0",
        name="stage carriage sits on lower guide",
    )
    ctx.expect_within(
        stage,
        stand,
        axes="x",
        inner_elem="stage_carriage",
        outer_elem="guide_bed",
        margin=0.001,
        name="stage carriage is centered on guide width",
    )
    with ctx.pose({stage_slide: -0.035}):
        rear_position = ctx.part_world_position(stage)
        ctx.expect_overlap(
            stage,
            stand,
            axes="y",
            elem_a="stage_carriage",
            elem_b="guide_bed",
            min_overlap=0.060,
            name="rear stage position remains retained on guide",
        )
    with ctx.pose({stage_slide: 0.035}):
        front_position = ctx.part_world_position(stage)
        ctx.expect_overlap(
            stage,
            stand,
            axes="y",
            elem_a="stage_carriage",
            elem_b="guide_bed",
            min_overlap=0.060,
            name="front stage position remains retained on guide",
        )
    ctx.check(
        "stage slide moves fore and aft",
        rear_position is not None
        and front_position is not None
        and front_position[1] > rear_position[1] + 0.060,
        details=f"rear={rear_position}, front={front_position}",
    )

    # A rotating turret moves the objective barrels around the fixed tube axis.
    rest_obj = aabb_center(ctx.part_element_world_aabb(nosepiece, elem="objective_0"))
    with ctx.pose({turret: math.pi / 2.0}):
        turned_obj = aabb_center(ctx.part_element_world_aabb(nosepiece, elem="objective_0"))
    ctx.check(
        "objective nosepiece rotates on turret",
        turned_obj[1] > rest_obj[1] + 0.020 and abs(turned_obj[0]) < 0.006,
        details=f"rest={rest_obj}, turned={turned_obj}",
    )

    # The lamp door is front-mounted under the stage and opens around a vertical
    # side hinge.
    ctx.expect_gap(
        lamp_door,
        stand,
        axis="y",
        min_gap=0.0,
        max_gap=0.006,
        positive_elem="door_panel",
        negative_elem="lamp_housing",
        name="lamp door closes against housing front",
    )
    ctx.expect_overlap(
        lamp_door,
        stand,
        axes="xz",
        elem_a="door_panel",
        elem_b="lamp_housing",
        min_overlap=0.040,
        name="lamp door covers the lamp access opening",
    )
    closed_door = aabb_center(ctx.part_element_world_aabb(lamp_door, elem="door_panel"))
    with ctx.pose({door_hinge: 1.10}):
        open_door = aabb_center(ctx.part_element_world_aabb(lamp_door, elem="door_panel"))
    ctx.check(
        "lamp access door swings outward on vertical side hinge",
        open_door[1] > closed_door[1] + 0.020 and open_door[0] < closed_door[0],
        details=f"closed={closed_door}, open={open_door}",
    )

    return ctx.report()


object_model = build_object_model()
