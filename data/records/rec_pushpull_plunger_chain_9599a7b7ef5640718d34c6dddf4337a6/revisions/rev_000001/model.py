from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rect_tube_mesh(
    *,
    length: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    corner: float,
    name: str,
):
    """A rounded rectangular sleeve/tube whose authored axis is local +X."""
    outer = rounded_rect_profile(outer_z, outer_y, corner, corner_segments=5)
    inner = rounded_rect_profile(inner_z, inner_y, max(corner * 0.55, 0.001), corner_segments=5)
    geom = ExtrudeWithHolesGeometry(outer, [inner], length, center=True)
    return mesh_from_geometry(geom, name)


def _add_rect_frame(
    part,
    *,
    prefix: str,
    depth_x: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    center_x: float,
    center_z: float,
    material,
    seam: float = 0.0015,
) -> None:
    """Build a through-opening retainer frame from four connected solid bars."""
    cap_z = (outer_z - inner_z) * 0.5 + seam
    side_y = (outer_y - inner_y) * 0.5 + seam
    part.visual(
        Box((depth_x, outer_y, cap_z)),
        origin=Origin(xyz=(center_x, 0.0, center_z + inner_z * 0.5 + cap_z * 0.5 - seam)),
        material=material,
        name=f"{prefix}_top",
    )
    part.visual(
        Box((depth_x, outer_y, cap_z)),
        origin=Origin(xyz=(center_x, 0.0, center_z - inner_z * 0.5 - cap_z * 0.5 + seam)),
        material=material,
        name=f"{prefix}_bottom",
    )
    for idx, y_sign in enumerate((-1.0, 1.0)):
        part.visual(
            Box((depth_x, side_y, inner_z + 2.0 * seam)),
            origin=Origin(
                xyz=(
                    center_x,
                    y_sign * (inner_y * 0.5 + side_y * 0.5 - seam),
                    center_z,
                )
            ),
            material=material,
            name=f"{prefix}_side_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_ejector_module")

    dark_cast = model.material("dark_cast_iron", rgba=(0.075, 0.083, 0.090, 1.0))
    blackened = model.material("blackened_steel", rgba=(0.015, 0.017, 0.018, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    polished = model.material("polished_plunger", rgba=(0.82, 0.84, 0.80, 1.0))
    bronze = model.material("oiled_bronze_wear", rgba=(0.62, 0.42, 0.18, 1.0))
    bolt = model.material("dark_socket_heads", rgba=(0.025, 0.025, 0.023, 1.0))

    tube_rpy = (0.0, math.pi / 2.0, 0.0)
    rod_rpy = (0.0, math.pi / 2.0, 0.0)
    axis_z = 0.18

    base = model.part("base_frame")
    base.visual(
        Box((0.95, 0.34, 0.055)),
        origin=Origin(xyz=(-0.19, 0.0, 0.0275)),
        material=dark_cast,
        name="ground_bed",
    )
    base.visual(
        _rect_tube_mesh(
            length=0.520,
            outer_y=0.180,
            outer_z=0.140,
            inner_y=0.138,
            inner_z=0.098,
            corner=0.012,
            name="guide_body_tube",
        ),
        origin=Origin(xyz=(-0.260, 0.0, axis_z), rpy=tube_rpy),
        material=blackened,
        name="guide_body",
    )
    _add_rect_frame(
        base,
        prefix="front_retainer",
        depth_x=0.032,
        outer_y=0.245,
        outer_z=0.205,
        inner_y=0.145,
        inner_z=0.105,
        center_x=0.012,
        center_z=axis_z,
        material=dark_cast,
        seam=0.002,
    )
    base.visual(
        Box((0.070, 0.250, 0.205)),
        origin=Origin(xyz=(-0.545, 0.0, 0.1275)),
        material=dark_cast,
        name="rear_stop_bulkhead",
    )
    for idx, y in enumerate((-0.098, 0.098)):
        base.visual(
            Box((0.500, 0.040, 0.122)),
            origin=Origin(xyz=(-0.255, y, 0.086)),
            material=dark_cast,
            name=f"guide_pedestal_{idx}",
        )
        base.visual(
            Box((0.760, 0.030, 0.028)),
            origin=Origin(xyz=(-0.185, y * 1.18, 0.070)),
            material=ground_steel,
            name=f"bed_rail_{idx}",
        )
    for idx, (y, z) in enumerate(
        (
            (-0.095, axis_z - 0.070),
            (0.095, axis_z - 0.070),
            (-0.105, axis_z),
            (0.105, axis_z),
            (-0.095, axis_z + 0.070),
            (0.095, axis_z + 0.070),
        )
    ):
        base.visual(
            Cylinder(radius=0.010, length=0.014),
            origin=Origin(xyz=(0.030, y, z), rpy=rod_rpy),
            material=bolt,
            name=f"retainer_bolt_{idx}",
        )

    sleeve_0 = model.part("outer_sleeve")
    sleeve_0.visual(
        _rect_tube_mesh(
            length=0.540,
            outer_y=0.118,
            outer_z=0.078,
            inner_y=0.084,
            inner_z=0.052,
            corner=0.008,
            name="outer_sleeve_tube_mesh",
        ),
        origin=Origin(xyz=(-0.190, 0.0, 0.0), rpy=tube_rpy),
        material=ground_steel,
        name="outer_sleeve_tube",
    )
    _add_rect_frame(
        sleeve_0,
        prefix="outer_stop",
        depth_x=0.026,
        outer_y=0.152,
        outer_z=0.106,
        inner_y=0.088,
        inner_z=0.056,
        center_x=0.075,
        center_z=0.0,
        material=blackened,
        seam=0.0015,
    )
    sleeve_1 = model.part("inner_sleeve")
    sleeve_1.visual(
        _rect_tube_mesh(
            length=0.440,
            outer_y=0.070,
            outer_z=0.042,
            inner_y=0.036,
            inner_z=0.026,
            corner=0.005,
            name="inner_sleeve_tube_mesh",
        ),
        origin=Origin(xyz=(-0.160, 0.0, 0.0), rpy=tube_rpy),
        material=ground_steel,
        name="inner_sleeve_tube",
    )
    _add_rect_frame(
        sleeve_1,
        prefix="inner_front_bushing",
        depth_x=0.024,
        outer_y=0.096,
        outer_z=0.066,
        inner_y=0.046,
        inner_z=0.032,
        center_x=0.058,
        center_z=0.0,
        material=bronze,
        seam=0.001,
    )
    for idx, y in enumerate((-0.026, 0.026)):
        sleeve_1.visual(
            Box((0.320, 0.010, 0.004)),
            origin=Origin(xyz=(-0.155, y, 0.0205)),
            material=bronze,
            name=f"inner_top_wear_{idx}",
        )
        sleeve_1.visual(
            Box((0.320, 0.010, 0.004)),
            origin=Origin(xyz=(-0.155, y, -0.0205)),
            material=bronze,
            name=f"inner_bottom_wear_{idx}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.011, length=0.395),
        origin=Origin(xyz=(-0.112, 0.0, 0.0), rpy=rod_rpy),
        material=polished,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=rod_rpy),
        material=polished,
        name="plunger_nose",
    )
    plunger.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(-0.305, 0.0, 0.0), rpy=rod_rpy),
        material=blackened,
        name="rear_rod_collar",
    )

    model.articulation(
        "base_to_outer_sleeve",
        ArticulationType.PRISMATIC,
        parent=base,
        child=sleeve_0,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.220),
    )
    model.articulation(
        "outer_to_inner_sleeve",
        ArticulationType.PRISMATIC,
        parent=sleeve_0,
        child=sleeve_1,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.32, lower=0.0, upper=0.170),
    )
    model.articulation(
        "inner_sleeve_to_plunger",
        ArticulationType.PRISMATIC,
        parent=sleeve_1,
        child=plunger,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=0.0, upper=0.150),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    outer = object_model.get_part("outer_sleeve")
    inner = object_model.get_part("inner_sleeve")
    plunger = object_model.get_part("plunger")
    j_outer = object_model.get_articulation("base_to_outer_sleeve")
    j_inner = object_model.get_articulation("outer_to_inner_sleeve")
    j_plunger = object_model.get_articulation("inner_sleeve_to_plunger")

    ctx.allow_overlap(
        base,
        outer,
        elem_a="guide_body",
        elem_b="outer_sleeve_tube",
        reason="The outer moving sleeve is intentionally retained inside the fixed guide bore.",
    )
    ctx.allow_overlap(
        base,
        inner,
        elem_a="guide_body",
        elem_b="inner_sleeve_tube",
        reason="The serial inner sleeve passes through the fixed guide bore while nested.",
    )
    ctx.allow_overlap(
        base,
        plunger,
        elem_a="guide_body",
        elem_b="plunger_rod",
        reason="The slim plunger rod is intentionally coaxial through the fixed guide bore.",
    )
    ctx.allow_overlap(
        inner,
        outer,
        elem_a="inner_sleeve_tube",
        elem_b="outer_sleeve_tube",
        reason="The inner sleeve is intentionally represented as sliding inside the outer sleeve guide.",
    )
    ctx.allow_overlap(
        outer,
        plunger,
        elem_a="outer_sleeve_tube",
        elem_b="plunger_rod",
        reason="The final rod passes through the outer sleeve's guide envelope while retracted.",
    )
    ctx.allow_overlap(
        inner,
        plunger,
        elem_a="inner_sleeve_tube",
        elem_b="plunger_rod",
        reason="The plunger rod is intentionally retained inside the final rectangular sleeve.",
    )

    ctx.expect_within(
        outer,
        base,
        axes="yz",
        inner_elem="outer_sleeve_tube",
        outer_elem="guide_body",
        margin=0.010,
        name="outer sleeve fits within fixed guide bore envelope",
    )
    ctx.expect_overlap(
        outer,
        base,
        axes="x",
        elem_a="outer_sleeve_tube",
        elem_b="guide_body",
        min_overlap=0.42,
        name="outer sleeve retained deeply in guide at retracted pose",
    )
    ctx.expect_within(
        inner,
        base,
        axes="yz",
        inner_elem="inner_sleeve_tube",
        outer_elem="guide_body",
        margin=0.010,
        name="inner sleeve also clears the fixed guide bore",
    )
    ctx.expect_overlap(
        inner,
        base,
        axes="x",
        elem_a="inner_sleeve_tube",
        elem_b="guide_body",
        min_overlap=0.28,
        name="inner sleeve visibly overlaps fixed guide when nested",
    )
    ctx.expect_within(
        plunger,
        base,
        axes="yz",
        inner_elem="plunger_rod",
        outer_elem="guide_body",
        margin=0.010,
        name="plunger rod is coaxial through the fixed guide",
    )
    ctx.expect_overlap(
        plunger,
        base,
        axes="x",
        elem_a="plunger_rod",
        elem_b="guide_body",
        min_overlap=0.16,
        name="plunger rod overlaps the fixed guide while retracted",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="yz",
        inner_elem="inner_sleeve_tube",
        outer_elem="outer_sleeve_tube",
        margin=0.006,
        name="inner sleeve centered inside outer sleeve",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        elem_a="inner_sleeve_tube",
        elem_b="outer_sleeve_tube",
        min_overlap=0.25,
        name="inner sleeve retained within outer sleeve at retracted pose",
    )
    ctx.expect_within(
        plunger,
        outer,
        axes="yz",
        inner_elem="plunger_rod",
        outer_elem="outer_sleeve_tube",
        margin=0.006,
        name="plunger rod clears the outer sleeve guide envelope",
    )
    ctx.expect_overlap(
        plunger,
        outer,
        axes="x",
        elem_a="plunger_rod",
        elem_b="outer_sleeve_tube",
        min_overlap=0.22,
        name="plunger rod is visibly nested through the outer sleeve",
    )
    ctx.expect_within(
        plunger,
        inner,
        axes="yz",
        inner_elem="plunger_rod",
        outer_elem="inner_sleeve_tube",
        margin=0.004,
        name="plunger rod remains within the final sleeve bore envelope",
    )
    ctx.expect_overlap(
        plunger,
        inner,
        axes="x",
        elem_a="plunger_rod",
        elem_b="inner_sleeve_tube",
        min_overlap=0.23,
        name="plunger rod has clear retained insertion when retracted",
    )
    rest_outer = ctx.part_world_position(outer)
    rest_inner = ctx.part_world_position(inner)
    rest_plunger = ctx.part_world_position(plunger)
    with ctx.pose({j_outer: 0.220, j_inner: 0.170, j_plunger: 0.150}):
        ctx.expect_overlap(
            outer,
            base,
            axes="x",
            elem_a="outer_sleeve_tube",
            elem_b="guide_body",
            min_overlap=0.20,
            name="outer sleeve retained in guide at full stroke",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            elem_a="inner_sleeve_tube",
            elem_b="outer_sleeve_tube",
            min_overlap=0.16,
            name="inner sleeve retained in outer sleeve at full stroke",
        )
        ctx.expect_overlap(
            plunger,
            inner,
            axes="x",
            elem_a="plunger_rod",
            elem_b="inner_sleeve_tube",
            min_overlap=0.12,
            name="plunger retained in final sleeve at full stroke",
        )
        extended_outer = ctx.part_world_position(outer)
        extended_inner = ctx.part_world_position(inner)
        extended_plunger = ctx.part_world_position(plunger)

    ctx.check(
        "all prismatic stages extend along the same front axis",
        rest_outer is not None
        and rest_inner is not None
        and rest_plunger is not None
        and extended_outer is not None
        and extended_inner is not None
        and extended_plunger is not None
        and extended_outer[0] > rest_outer[0] + 0.20
        and extended_inner[0] > rest_inner[0] + 0.37
        and extended_plunger[0] > rest_plunger[0] + 0.52,
        details=(
            f"rest outer/inner/plunger={rest_outer},{rest_inner},{rest_plunger}; "
            f"extended={extended_outer},{extended_inner},{extended_plunger}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
