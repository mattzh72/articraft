from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _table_plate_mesh():
    """Rounded cast-iron work table with a through drill hole and recessed slots."""
    plate = cq.Workplane("XY").box(0.52, 0.36, 0.04)
    plate = plate.edges("|Z").fillet(0.018)
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .circle(0.034)
        .cutThruAll()
    )

    # Shallow machined work-holding slots.  They leave the table as one solid
    # casting instead of cutting it into separate bars.
    for y in (-0.095, 0.095):
        cutter = cq.Workplane("XY").box(0.42, 0.026, 0.020).translate((0.0, y, 0.025))
        plate = plate.cut(cutter)

    # Small front relief slot for chips / clamp access.
    cutter = cq.Workplane("XY").box(0.026, 0.18, 0.018).translate((0.13, 0.0, 0.026))
    plate = plate.cut(cutter)
    return mesh_from_cadquery(plate, "machined_table_plate", tolerance=0.0008)


def _sector_plate_mesh():
    """Thin annular quadrant plate for the fixed tilt angle scale."""
    inner_r = 0.112
    outer_r = 0.148
    thickness = 0.010
    start = math.radians(-55.0)
    end = math.radians(55.0)
    steps = 28

    geom = MeshGeometry()
    outer_front = []
    outer_back = []
    inner_front = []
    inner_back = []
    for i in range(steps + 1):
        t = start + (end - start) * i / steps
        c = math.cos(t)
        s = math.sin(t)
        outer_front.append(geom.add_vertex(outer_r * c, -thickness / 2.0, outer_r * s))
        outer_back.append(geom.add_vertex(outer_r * c, thickness / 2.0, outer_r * s))
        inner_front.append(geom.add_vertex(inner_r * c, -thickness / 2.0, inner_r * s))
        inner_back.append(geom.add_vertex(inner_r * c, thickness / 2.0, inner_r * s))

    for i in range(steps):
        # Front and back annular faces.
        geom.add_face(outer_front[i], outer_front[i + 1], inner_front[i + 1])
        geom.add_face(outer_front[i], inner_front[i + 1], inner_front[i])
        geom.add_face(outer_back[i + 1], outer_back[i], inner_back[i])
        geom.add_face(outer_back[i + 1], inner_back[i], inner_back[i + 1])
        # Outer and inner curved edges.
        geom.add_face(outer_front[i + 1], outer_front[i], outer_back[i])
        geom.add_face(outer_front[i + 1], outer_back[i], outer_back[i + 1])
        geom.add_face(inner_front[i], inner_front[i + 1], inner_back[i + 1])
        geom.add_face(inner_front[i], inner_back[i + 1], inner_back[i])

    # End caps.
    for i in (0, steps):
        geom.add_face(outer_front[i], inner_front[i], inner_back[i])
        geom.add_face(outer_front[i], inner_back[i], outer_back[i])

    return mesh_from_geometry(geom, "tilt_sector_plate")


def _lobed_knob(name: str, diameter: float, height: float):
    return mesh_from_geometry(
        KnobGeometry(
            diameter,
            height,
            body_style="lobed",
            base_diameter=diameter * 0.68,
            top_diameter=diameter * 0.92,
            crown_radius=0.0018,
            grip=KnobGrip(style="ribbed", count=10, depth=0.0012),
            bore=KnobBore(style="round", diameter=diameter * 0.20),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drill_press_tilt_table")

    cast_iron = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.11, 1.0))
    worn_iron = model.material("worn_machined_iron", rgba=(0.22, 0.23, 0.22, 1.0))
    column_steel = model.material("brushed_column_steel", rgba=(0.47, 0.49, 0.48, 1.0))
    black = model.material("black_handwheel", rgba=(0.015, 0.014, 0.013, 1.0))
    bolt = model.material("oiled_bolt_steel", rgba=(0.33, 0.32, 0.30, 1.0))
    white = model.material("engraved_white", rgba=(0.90, 0.88, 0.78, 1.0))
    red = model.material("red_angle_pointer", rgba=(0.75, 0.04, 0.03, 1.0))

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.045, length=0.78),
        origin=Origin(xyz=(-0.34, 0.0, -0.08)),
        material=column_steel,
        name="column_tube",
    )
    mount.visual(
        Box((0.17, 0.17, 0.095)),
        origin=Origin(xyz=(-0.34, 0.0, -0.08)),
        material=cast_iron,
        name="split_collar_body",
    )
    mount.visual(
        Box((0.012, 0.004, 0.088)),
        origin=Origin(xyz=(-0.34, -0.087, -0.08)),
        material=black,
        name="collar_split_line",
    )
    mount.visual(
        Box((0.13, 0.038, 0.085)),
        origin=Origin(xyz=(-0.34, -0.104, -0.08)),
        material=cast_iron,
        name="column_clamp_lug",
    )
    mount.visual(
        Box((0.39, 0.082, 0.072)),
        origin=Origin(xyz=(-0.155, 0.0, -0.085)),
        material=cast_iron,
        name="support_arm",
    )
    mount.visual(
        Box((0.27, 0.045, 0.044)),
        origin=Origin(xyz=(-0.09, 0.105, -0.112), rpy=(0.0, -0.20, 0.0)),
        material=cast_iron,
        name="upper_gusset_0",
    )
    mount.visual(
        Box((0.27, 0.045, 0.044)),
        origin=Origin(xyz=(-0.09, -0.105, -0.112), rpy=(0.0, -0.20, 0.0)),
        material=cast_iron,
        name="upper_gusset_1",
    )
    mount.visual(
        Box((0.19, 0.57, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=cast_iron,
        name="yoke_base",
    )
    mount.visual(
        Box((0.14, 0.040, 0.18)),
        origin=Origin(xyz=(0.0, 0.250, 0.0)),
        material=cast_iron,
        name="lock_yoke_cheek",
    )
    mount.visual(
        Box((0.14, 0.040, 0.18)),
        origin=Origin(xyz=(0.0, -0.250, 0.0)),
        material=cast_iron,
        name="scale_yoke_cheek",
    )
    mount.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(0.0, 0.226, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bolt,
        name="lock_side_bushing",
    )
    mount.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(0.0, -0.226, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt,
        name="scale_side_bushing",
    )
    mount.visual(
        _sector_plate_mesh(),
        origin=Origin(xyz=(0.0, -0.224, 0.0)),
        material=worn_iron,
        name="tilt_sector_plate",
    )
    mount.visual(
        Box((0.055, 0.014, 0.040)),
        origin=Origin(xyz=(0.083, -0.226, 0.0)),
        material=cast_iron,
        name="sector_mount_tab",
    )
    for idx, angle_deg in enumerate(range(-45, 46, 15)):
        angle = math.radians(angle_deg)
        radius = 0.133
        tick_len = 0.023 if angle_deg in (-45, 0, 45) else 0.015
        mount.visual(
            Box((0.0035, 0.005, tick_len)),
            origin=Origin(
                xyz=(radius * math.cos(angle), -0.230, radius * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=white,
            name=f"tilt_tick_{idx}",
        )

    table = model.part("table")
    table.visual(
        _table_plate_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=worn_iron,
        name="table_plate",
    )
    table.visual(
        Box((0.43, 0.038, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=cast_iron,
        name="underside_rib_x",
    )
    table.visual(
        Box((0.052, 0.32, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=cast_iron,
        name="underside_rib_y",
    )
    table.visual(
        Box((0.36, 0.028, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.022), rpy=(0.0, 0.0, math.radians(30))),
        material=cast_iron,
        name="diagonal_rib_0",
    )
    table.visual(
        Box((0.36, 0.028, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.022), rpy=(0.0, 0.0, math.radians(-30))),
        material=cast_iron,
        name="diagonal_rib_1",
    )
    table.visual(
        Cylinder(radius=0.038, length=0.57),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bolt,
        name="trunnion_barrel",
    )
    table.visual(
        Cylinder(radius=0.052, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="central_trunnion_boss",
    )
    table.visual(
        Box((0.110, 0.008, 0.010)),
        origin=Origin(xyz=(0.085, -0.288, 0.0)),
        material=red,
        name="angle_pointer",
    )

    tilt_lock = model.part("tilt_lock")
    tilt_lock.visual(
        Cylinder(radius=0.011, length=0.052),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bolt,
        name="tilt_lock_screw",
    )
    tilt_lock.visual(
        Cylinder(radius=0.024, length=0.009),
        origin=Origin(xyz=(0.0, 0.053, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bolt,
        name="tilt_lock_washer",
    )
    tilt_lock.visual(
        _lobed_knob("tilt_lock_knob_mesh", 0.070, 0.040),
        origin=Origin(xyz=(0.0, 0.075, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="tilt_lock_knob",
    )

    column_lock = model.part("column_lock")
    column_lock.visual(
        Cylinder(radius=0.010, length=0.058),
        origin=Origin(xyz=(0.0, -0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt,
        name="column_lock_screw",
    )
    column_lock.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt,
        name="column_lock_washer",
    )
    column_lock.visual(
        _lobed_knob("column_lock_knob_mesh", 0.060, 0.036),
        origin=Origin(xyz=(0.0, -0.078, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="column_lock_knob",
    )

    model.articulation(
        "mount_to_table",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=table,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=-math.radians(45), upper=math.radians(45)),
        motion_properties=MotionProperties(damping=2.5, friction=3.0),
    )
    model.articulation(
        "table_to_tilt_lock",
        ArticulationType.CONTINUOUS,
        parent=table,
        child=tilt_lock,
        origin=Origin(xyz=(0.0, 0.285, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=5.0),
        motion_properties=MotionProperties(damping=0.25, friction=0.55),
    )
    model.articulation(
        "mount_to_column_lock",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=column_lock,
        origin=Origin(xyz=(-0.34, -0.123, -0.08)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
        motion_properties=MotionProperties(damping=0.2, friction=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    table = object_model.get_part("table")
    tilt = object_model.get_articulation("mount_to_table")
    tilt_lock_joint = object_model.get_articulation("table_to_tilt_lock")
    column_lock_joint = object_model.get_articulation("mount_to_column_lock")

    ctx.allow_overlap(
        mount,
        table,
        elem_a="lock_yoke_cheek",
        elem_b="trunnion_barrel",
        reason="The tilting trunnion is intentionally captured in the lock-side yoke bearing.",
    )
    ctx.allow_overlap(
        mount,
        table,
        elem_a="scale_yoke_cheek",
        elem_b="trunnion_barrel",
        reason="The tilting trunnion is intentionally captured in the scale-side yoke bearing.",
    )
    ctx.allow_overlap(
        mount,
        table,
        elem_a="lock_side_bushing",
        elem_b="trunnion_barrel",
        reason="The bronze/steel bearing bushing is represented as a solid collar around the captured trunnion.",
    )
    ctx.allow_overlap(
        mount,
        table,
        elem_a="scale_side_bushing",
        elem_b="trunnion_barrel",
        reason="The bronze/steel bearing bushing is represented as a solid collar around the captured trunnion.",
    )

    ctx.expect_overlap(
        table,
        mount,
        axes="y",
        elem_a="trunnion_barrel",
        elem_b="lock_yoke_cheek",
        min_overlap=0.030,
        name="lock-side trunnion remains seated in yoke",
    )
    ctx.expect_overlap(
        table,
        mount,
        axes="y",
        elem_a="trunnion_barrel",
        elem_b="scale_yoke_cheek",
        min_overlap=0.030,
        name="scale-side trunnion remains seated in yoke",
    )
    ctx.expect_overlap(
        table,
        mount,
        axes="xz",
        elem_a="trunnion_barrel",
        elem_b="lock_side_bushing",
        min_overlap=0.050,
        name="lock-side bushing is concentric with trunnion",
    )
    ctx.expect_overlap(
        table,
        mount,
        axes="xz",
        elem_a="trunnion_barrel",
        elem_b="scale_side_bushing",
        min_overlap=0.050,
        name="scale-side bushing is concentric with trunnion",
    )
    ctx.expect_gap(
        table,
        mount,
        axis="z",
        positive_elem="table_plate",
        negative_elem="yoke_base",
        min_gap=0.10,
        name="flat work surface clears the lower support casting",
    )

    limits = tilt.motion_limits
    ctx.check(
        "tilt table has workshop-angle limits",
        limits is not None and limits.lower <= -0.78 and limits.upper >= 0.78,
        details=f"limits={limits}",
    )
    ctx.check(
        "locking knobs spin on screw axes",
        tilt_lock_joint.articulation_type == ArticulationType.CONTINUOUS
        and column_lock_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"tilt_lock={tilt_lock_joint.articulation_type}, column_lock={column_lock_joint.articulation_type}",
    )

    rest_aabb = ctx.part_world_aabb(table)
    with ctx.pose({tilt: math.radians(30)}):
        tilted_aabb = ctx.part_world_aabb(table)
        ctx.expect_overlap(
            table,
            mount,
            axes="y",
            elem_a="trunnion_barrel",
            elem_b="lock_yoke_cheek",
            min_overlap=0.030,
            name="tilted trunnion stays captured",
        )
    ctx.check(
        "tilt pose visibly changes table height envelope",
        rest_aabb is not None
        and tilted_aabb is not None
        and abs(tilted_aabb[1][2] - rest_aabb[1][2]) > 0.025,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
