from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


SHAFT_R = 0.014
BEARING_HOLE_R = SHAFT_R


def _bevel_gear_geometry(
    *,
    teeth: int = 20,
    face_width: float = 0.060,
    broad_radius: float = 0.074,
    small_radius: float = 0.026,
    tooth_depth: float = 0.008,
    small_end_positive: bool = True,
) -> MeshGeometry:
    """A light-weight faceted bevel gear: a tapered body with raised radial teeth."""

    segments = teeth * 4
    geom = MeshGeometry()
    rings: list[list[int]] = []
    z_values = (-face_width / 2.0, face_width / 2.0)

    for z in z_values:
        t = (z + face_width / 2.0) / face_width
        if small_end_positive:
            root_r = broad_radius * (1.0 - t) + small_radius * t
        else:
            root_r = small_radius * (1.0 - t) + broad_radius * t

        ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            # Four samples per tooth: valley, rising flank, crest, falling flank.
            tooth_phase = (i % 4) / 4.0
            if tooth_phase == 0.0:
                profile = 0.0
            elif tooth_phase == 0.25:
                profile = 0.65
            elif tooth_phase == 0.50:
                profile = 1.0
            else:
                profile = 0.65
            radius = root_r + tooth_depth * profile
            ring.append(geom.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
        rings.append(ring)

    # Tapered tooth flanks.
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(rings[0][i], rings[0][j], rings[1][j])
        geom.add_face(rings[0][i], rings[1][j], rings[1][i])

    # Solid end caps; the mounted shaft and hub visually lock into the center.
    bottom_center = geom.add_vertex(0.0, 0.0, z_values[0])
    top_center = geom.add_vertex(0.0, 0.0, z_values[1])
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(bottom_center, rings[0][j], rings[0][i])
        geom.add_face(top_center, rings[1][i], rings[1][j])

    return geom


def _bearing_block(size: tuple[float, float, float], axis: str) -> object:
    """Transparent pillow-block body with a real through hole for the shaft."""

    sx, sy, sz = size
    block = cq.Workplane("XY").box(sx, sy, sz)
    if axis == "z":
        cutter = cq.Workplane("XY").circle(BEARING_HOLE_R).extrude(sz * 1.6, both=True)
    elif axis == "x":
        cutter = cq.Workplane("YZ").circle(BEARING_HOLE_R).extrude(sx * 1.6, both=True)
    else:
        raise ValueError(f"unsupported bearing axis {axis!r}")
    return block.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="right_angle_bevel_gear_assembly")

    model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("dark_edge", rgba=(0.05, 0.055, 0.06, 1.0))
    model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    model.material("oiled_bronze", rgba=(0.82, 0.55, 0.20, 1.0))
    model.material("clear_bearing", rgba=(0.50, 0.80, 1.00, 0.32))

    base = model.part("base_bracket")
    base.visual(
        Box((0.56, 0.30, 0.025)),
        origin=Origin(xyz=(0.20, 0.0, 0.0125)),
        material="painted_steel",
        name="base_plate",
    )
    base.visual(
        Box((0.055, 0.025, 0.205)),
        origin=Origin(xyz=(0.0, 0.075, 0.1275)),
        material="painted_steel",
        name="vertical_yoke_0",
    )
    base.visual(
        Box((0.055, 0.025, 0.205)),
        origin=Origin(xyz=(0.0, -0.075, 0.1275)),
        material="painted_steel",
        name="vertical_yoke_1",
    )

    vertical_lower_bearing_size = (0.095, 0.130, 0.060)
    vertical_upper_bearing_size = (0.095, 0.130, 0.060)
    horizontal_bearing_size = (0.060, 0.105, 0.085)
    for name, z, size in (
        ("clear_vertical_bearing_0", 0.055, vertical_lower_bearing_size),
        ("clear_vertical_bearing_1", 0.205, vertical_upper_bearing_size),
    ):
        base.visual(
            mesh_from_cadquery(_bearing_block(size, "z"), name),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="clear_bearing",
            name=name,
        )

    for x in (0.22, 0.37):
        base.visual(
            Box((0.060, 0.085, 0.263)),
            origin=Origin(xyz=(x, 0.0, 0.1565)),
            material="painted_steel",
            name=f"horizontal_pedestal_{0 if x < 0.3 else 1}",
        )
    for idx, x in enumerate((0.22, 0.37)):
        name = f"clear_horizontal_bearing_{idx}"
        base.visual(
            mesh_from_cadquery(_bearing_block(horizontal_bearing_size, "x"), name),
            origin=Origin(xyz=(x, 0.0, 0.330)),
            material="clear_bearing",
            name=name,
        )

    # Small cap screws make the clear blocks read as bolted bearing blocks.
    for x, z, prefix in ((0.0, 0.055, "vertical_lower"), (0.0, 0.205, "vertical_upper")):
        for yi, y in enumerate((-0.048, 0.048)):
            for xi, dx in enumerate((-0.030, 0.030)):
                base.visual(
                    Cylinder(radius=0.006, length=0.004),
                    origin=Origin(xyz=(x + dx, y, z + 0.032), rpy=(0.0, 0.0, 0.0)),
                    material="dark_edge",
                    name=f"{prefix}_bolt_{yi}_{xi}",
                )
    for block_i, x in enumerate((0.22, 0.37)):
        for yi, y in enumerate((-0.038, 0.038)):
            for zi, dz in enumerate((-0.025, 0.025)):
                base.visual(
                    Cylinder(radius=0.005, length=0.004),
                    origin=Origin(xyz=(x - 0.032, y, 0.330 + dz), rpy=(0.0, math.pi / 2.0, 0.0)),
                    material="dark_edge",
                    name=f"horizontal_bolt_{block_i}_{yi}_{zi}",
                )

    vertical = model.part("vertical_shaft")
    vertical.visual(
        Cylinder(radius=SHAFT_R, length=0.300),
        origin=Origin(),
        material="brushed_steel",
        name="shaft",
    )
    vertical.visual(
        mesh_from_geometry(
            _bevel_gear_geometry(teeth=22, small_end_positive=True),
            "vertical_bevel_gear",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material="oiled_bronze",
        name="bevel_gear",
    )
    vertical.visual(
        Cylinder(radius=0.030, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material="oiled_bronze",
        name="gear_hub",
    )

    horizontal = model.part("horizontal_shaft")
    horizontal.visual(
        Cylinder(radius=SHAFT_R, length=0.345),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="shaft",
    )
    horizontal.visual(
        mesh_from_geometry(
            _bevel_gear_geometry(teeth=22, small_end_positive=False),
            "horizontal_bevel_gear",
        ),
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="oiled_bronze",
        name="bevel_gear",
    )
    horizontal.visual(
        Cylinder(radius=0.030, length=0.035),
        origin=Origin(xyz=(-0.175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="oiled_bronze",
        name="gear_hub",
    )

    model.articulation(
        "vertical_shaft_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=vertical,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=10.0, velocity=8.0),
    )
    model.articulation(
        "horizontal_shaft_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=horizontal,
        origin=Origin(xyz=(0.270, 0.0, 0.330)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=10.0, velocity=8.0),
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

    base = object_model.get_part("base_bracket")
    vertical = object_model.get_part("vertical_shaft")
    horizontal = object_model.get_part("horizontal_shaft")
    vertical_joint = object_model.get_articulation("vertical_shaft_joint")
    horizontal_joint = object_model.get_articulation("horizontal_shaft_joint")

    for bearing_name in ("clear_vertical_bearing_0", "clear_vertical_bearing_1"):
        ctx.allow_overlap(
            base,
            vertical,
            elem_a=bearing_name,
            elem_b="shaft",
            reason=(
                "The transparent bearing block is a simplified clear bushing proxy; "
                "the vertical shaft is intentionally shown captured through it."
            ),
        )
    for bearing_name in ("clear_horizontal_bearing_0", "clear_horizontal_bearing_1"):
        ctx.allow_overlap(
            base,
            horizontal,
            elem_a=bearing_name,
            elem_b="shaft",
            reason=(
                "The transparent bearing block is a simplified clear bushing proxy; "
                "the horizontal shaft is intentionally shown captured through it."
            ),
        )

    ctx.check(
        "vertical shaft is revolute about Z",
        vertical_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(vertical_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={vertical_joint.articulation_type}, axis={vertical_joint.axis}",
    )
    ctx.check(
        "horizontal shaft is revolute about X",
        horizontal_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(horizontal_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={horizontal_joint.articulation_type}, axis={horizontal_joint.axis}",
    )

    ctx.expect_within(
        vertical,
        base,
        axes="xy",
        inner_elem="shaft",
        outer_elem="clear_vertical_bearing_0",
        margin=0.002,
        name="vertical shaft runs through lower bearing footprint",
    )
    ctx.expect_overlap(
        vertical,
        base,
        axes="z",
        elem_a="shaft",
        elem_b="clear_vertical_bearing_1",
        min_overlap=0.040,
        name="vertical shaft passes through upper bearing",
    )
    ctx.expect_within(
        horizontal,
        base,
        axes="yz",
        inner_elem="shaft",
        outer_elem="clear_horizontal_bearing_0",
        margin=0.002,
        name="horizontal shaft runs through bearing footprint",
    )
    ctx.expect_overlap(
        horizontal,
        base,
        axes="x",
        elem_a="shaft",
        elem_b="clear_horizontal_bearing_1",
        min_overlap=0.045,
        name="horizontal shaft passes through outer bearing",
    )
    ctx.expect_overlap(
        vertical,
        horizontal,
        axes="yz",
        elem_a="bevel_gear",
        elem_b="bevel_gear",
        min_overlap=0.020,
        name="bevel gears share the right-angle mesh region",
    )

    vertical_rest = ctx.part_world_position(vertical)
    horizontal_rest = ctx.part_world_position(horizontal)
    with ctx.pose({vertical_joint: 0.75, horizontal_joint: -0.60}):
        vertical_turned = ctx.part_world_position(vertical)
        horizontal_turned = ctx.part_world_position(horizontal)
    ctx.check(
        "both shafts spin in their bearings without translating",
        vertical_rest == vertical_turned and horizontal_rest == horizontal_turned,
        details=(
            f"vertical rest/turned={vertical_rest}/{vertical_turned}, "
            f"horizontal rest/turned={horizontal_rest}/{horizontal_turned}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
