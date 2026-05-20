from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 48,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    angles = [2.0 * math.pi * i / segments for i in range(segments)]
    if clockwise:
        angles = list(reversed(angles))
    cx, cy = center
    return [(cx + radius * math.cos(a), cy + radius * math.sin(a)) for a in angles]


def _cheek_profile(
    *,
    half_width: float,
    bottom_z: float,
    shaft_z: float,
    arc_segments: int = 18,
) -> list[tuple[float, float]]:
    """Tombstone cheek profile in local (Y, Z), before extrusion through X."""
    points: list[tuple[float, float]] = [
        (-half_width, bottom_z),
        (half_width, bottom_z),
        (half_width, shaft_z),
    ]
    for i in range(1, arc_segments):
        angle = math.pi * i / arc_segments
        points.append(
            (
                half_width * math.cos(angle),
                shaft_z + half_width * math.sin(angle),
            )
        )
    points.append((-half_width, shaft_z))
    return points


def _remap_yz_extrusion(geom: MeshGeometry, *, x_offset: float) -> MeshGeometry:
    """Map an extrusion whose local XY profile is YZ into world XYZ."""
    mapped = MeshGeometry()
    for local_x, local_y, local_z in geom.vertices:
        mapped.add_vertex(x_offset + local_z, local_x, local_y)
    for a, b, c in geom.faces:
        mapped.add_face(a, b, c)
    return mapped


def _side_cheek_mesh(*, x_offset: float, shaft_z: float) -> MeshGeometry:
    outer = _cheek_profile(half_width=0.19, bottom_z=0.045, shaft_z=shaft_z)
    hole = _circle_profile(
        0.047,
        center=(0.0, shaft_z),
        segments=48,
        clockwise=True,
    )
    return _remap_yz_extrusion(
        ExtrudeWithHolesGeometry(outer, [hole], 0.055, center=True),
        x_offset=x_offset,
    )


def _bearing_ring_mesh(*, x_offset: float, shaft_z: float) -> MeshGeometry:
    outer = _circle_profile(0.078, center=(0.0, shaft_z), segments=64)
    bore = _circle_profile(
        0.045,
        center=(0.0, shaft_z),
        segments=48,
        clockwise=True,
    )
    return _remap_yz_extrusion(
        ExtrudeWithHolesGeometry(outer, [bore], 0.076, center=True),
        x_offset=x_offset,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_table_module")

    dark_iron = model.material("dark_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    parkerized = model.material("parkerized_steel", rgba=(0.20, 0.23, 0.25, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.12, 0.20, 0.28, 1.0))
    machined = model.material("machined_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    slot_dark = model.material("slot_dark", rgba=(0.015, 0.017, 0.018, 1.0))

    foundation = model.part("foundation")
    foundation.visual(
        Cylinder(radius=0.40, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_iron,
        name="floor_plinth",
    )
    foundation.visual(
        Cylinder(radius=0.300, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=parkerized,
        name="bearing_track",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        foundation.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(
                xyz=(
                    0.315 * math.cos(angle),
                    0.315 * math.sin(angle),
                    0.082,
                )
            ),
            material=slot_dark,
            name=f"mount_bolt_{index}",
        )

    shaft_z = 0.300

    base_stage = model.part("base_stage")
    base_stage.visual(
        Cylinder(radius=0.320, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=blued_steel,
        name="turntable_disc",
    )
    base_stage.visual(
        Cylinder(radius=0.255, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=parkerized,
        name="top_step",
    )
    for index, x_offset in enumerate((-0.280, 0.280)):
        base_stage.visual(
            mesh_from_geometry(
                _side_cheek_mesh(x_offset=x_offset, shaft_z=shaft_z),
                f"side_cheek_{index}",
            ),
            material=blued_steel,
            name=f"cheek_{index}",
        )
        base_stage.visual(
            mesh_from_geometry(
                _bearing_ring_mesh(x_offset=x_offset, shaft_z=shaft_z),
                f"bearing_ring_{index}",
            ),
            material=parkerized,
            name=f"bearing_ring_{index}",
        )
    for index, y_offset in enumerate((-0.215, 0.215)):
        base_stage.visual(
            Box((0.610, 0.035, 0.065)),
            origin=Origin(xyz=(0.0, y_offset, 0.110)),
            material=blued_steel,
            name=f"tie_bar_{index}",
        )

    workplate = model.part("workplate")
    workplate.visual(
        Cylinder(radius=0.200, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined,
        name="plate",
    )
    workplate.visual(
        Cylinder(radius=0.028, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=parkerized,
        name="shaft",
    )
    workplate.visual(
        Cylinder(radius=0.072, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0395)),
        material=machined,
        name="center_hub",
    )
    for index, x_offset in enumerate((-0.075, 0.075)):
        workplate.visual(
            Box((0.018, 0.340, 0.006)),
            origin=Origin(xyz=(x_offset, 0.0, 0.025)),
            material=slot_dark,
            name=f"long_slot_{index}",
        )
    workplate.visual(
        Box((0.330, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=slot_dark,
        name="cross_slot",
    )
    for index in range(6):
        angle = 2.0 * math.pi * index / 6.0
        workplate.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(
                xyz=(
                    0.140 * math.cos(angle),
                    0.140 * math.sin(angle),
                    0.025,
                )
            ),
            material=slot_dark,
            name=f"bolt_hole_{index}",
        )

    model.articulation(
        "foundation_to_base",
        ArticulationType.REVOLUTE,
        parent=foundation,
        child=base_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "base_to_workplate",
        ArticulationType.REVOLUTE,
        parent=base_stage,
        child=workplate,
        origin=Origin(xyz=(0.0, 0.0, shaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.0,
            lower=-1.05,
            upper=1.05,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foundation = object_model.get_part("foundation")
    base_stage = object_model.get_part("base_stage")
    workplate = object_model.get_part("workplate")
    base_joint = object_model.get_articulation("foundation_to_base")
    trunnion_joint = object_model.get_articulation("base_to_workplate")

    for index in (0, 1):
        ctx.allow_overlap(
            base_stage,
            workplate,
            elem_a=f"bearing_ring_{index}",
            elem_b="shaft",
            reason="The transverse shaft is intentionally captured through the trunnion bearing bore.",
        )
        ctx.allow_overlap(
            base_stage,
            workplate,
            elem_a=f"cheek_{index}",
            elem_b="shaft",
            reason="The shaft passes through the bored side cheek that supports the trunnion.",
        )

    ctx.check(
        "base joint is vertical revolute",
        base_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(base_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={base_joint.articulation_type}, axis={base_joint.axis}",
    )
    ctx.check(
        "workplate joint is horizontal trunnion",
        trunnion_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(trunnion_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={trunnion_joint.articulation_type}, axis={trunnion_joint.axis}",
    )

    ctx.expect_gap(
        base_stage,
        foundation,
        axis="z",
        positive_elem="turntable_disc",
        negative_elem="bearing_track",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on bearing track",
    )
    ctx.expect_overlap(
        base_stage,
        foundation,
        axes="xy",
        elem_a="turntable_disc",
        elem_b="bearing_track",
        min_overlap=0.250,
        name="round base is centered on bearing track",
    )

    ctx.expect_gap(
        base_stage,
        workplate,
        axis="x",
        positive_elem="cheek_1",
        negative_elem="plate",
        min_gap=0.040,
        name="plate clears positive cheek",
    )
    ctx.expect_gap(
        workplate,
        base_stage,
        axis="x",
        positive_elem="plate",
        negative_elem="cheek_0",
        min_gap=0.040,
        name="plate clears negative cheek",
    )
    for index in (0, 1):
        ctx.expect_overlap(
            workplate,
            base_stage,
            axes="x",
            elem_a="shaft",
            elem_b=f"bearing_ring_{index}",
            min_overlap=0.030,
            name=f"shaft passes through bearing {index}",
        )
        ctx.expect_within(
            workplate,
            base_stage,
            axes="yz",
            inner_elem="shaft",
            outer_elem=f"bearing_ring_{index}",
            margin=0.0,
            name=f"shaft is centered in bearing {index}",
        )
        ctx.expect_overlap(
            workplate,
            base_stage,
            axes="x",
            elem_a="shaft",
            elem_b=f"cheek_{index}",
            min_overlap=0.030,
            name=f"shaft passes through cheek {index}",
        )
        ctx.expect_within(
            workplate,
            base_stage,
            axes="yz",
            inner_elem="shaft",
            outer_elem=f"cheek_{index}",
            margin=0.0,
            name=f"shaft is centered in cheek {index}",
        )

    rest_aabb = ctx.part_world_aabb(workplate)
    with ctx.pose({trunnion_joint: 0.75}):
        tilted_aabb = ctx.part_world_aabb(workplate)
        ctx.expect_gap(
            workplate,
            base_stage,
            axis="z",
            positive_elem="plate",
            negative_elem="turntable_disc",
            min_gap=0.030,
            name="tilted workplate clears turntable",
        )
    ctx.check(
        "trunnion pose changes plate height envelope",
        rest_aabb is not None
        and tilted_aabb is not None
        and (tilted_aabb[1][2] - rest_aabb[1][2]) > 0.030,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
