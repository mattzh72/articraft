from __future__ import annotations

import math

import cadquery as cq

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


def _cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder helper: SDK cylinders are local-Z, this turns them along +X."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder helper: SDK cylinders are local-Z, this turns them along Y."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _tapered_beam(
    *,
    length: float,
    width_start: float,
    width_end: float,
    height_start: float,
    height_end: float,
) -> cq.Workplane:
    """Loft a rectangular beam from a heavy root end to a lighter tip end."""
    return (
        cq.Workplane("YZ")
        .rect(width_start, height_start)
        .workplane(offset=length)
        .rect(width_end, height_end)
        .loft()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_transfer_arm")

    cast = model.material("dark_cast_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    plate_mat = model.material("powder_coat_plate", rgba=(0.18, 0.20, 0.22, 1.0))
    link_mat = model.material("blue_grey_link", rgba=(0.28, 0.36, 0.42, 1.0))
    forearm_mat = model.material("light_grey_forearm", rgba=(0.45, 0.50, 0.52, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    black = model.material("black_fastener", rgba=(0.02, 0.02, 0.018, 1.0))

    wall_mount = model.part("wall_mount")

    # Heavy backplate and shoulder support package.  The part frame is the
    # shoulder pivot axis; the wall is behind the assembly on negative X.
    wall_mount.visual(
        Box((0.080, 0.520, 0.720)),
        origin=Origin(xyz=(-0.240, 0.0, 0.0)),
        material=plate_mat,
        name="wall_backplate",
    )
    wall_mount.visual(
        Box((0.110, 0.360, 0.220)),
        origin=Origin(xyz=(-0.150, 0.0, 0.0)),
        material=cast,
        name="shoulder_rear_bridge",
    )
    for side, (y, cheek_name) in enumerate(
        ((-0.155, "shoulder_cheek_0"), (0.155, "shoulder_cheek_1"))
    ):
        wall_mount.visual(
            Box((0.230, 0.050, 0.280)),
            origin=Origin(xyz=(-0.020, y, 0.0)),
            material=cast,
            name=cheek_name,
        )
        wall_mount.visual(
            Box((0.240, 0.035, 0.052)),
            origin=Origin(xyz=(-0.135, y * 1.28, -0.155), rpy=(0.0, -0.46, 0.0)),
            material=cast,
            name=f"lower_gusset_{side}",
        )
        wall_mount.visual(
            Box((0.205, 0.032, 0.045)),
            origin=Origin(xyz=(-0.125, y * 1.28, 0.150), rpy=(0.0, 0.34, 0.0)),
            material=cast,
            name=f"upper_gusset_{side}",
        )
        # Cheek clamp bolts on the outside of the split shoulder housing.
        for z in (-0.078, 0.078):
            bolt_geom, bolt_origin = _cyl_y(0.012, 0.014)
            wall_mount.visual(
                bolt_geom,
                origin=Origin(
                    xyz=(-0.026, y + (0.028 if y > 0 else -0.028), z),
                    rpy=bolt_origin.rpy,
                ),
                material=black,
                name=f"shoulder_bolt_{side}_{'top' if z > 0 else 'bottom'}",
            )

    shoulder_pin_geom, shoulder_pin_origin = _cyl_y(0.024, 0.370)
    wall_mount.visual(
        shoulder_pin_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=shoulder_pin_origin.rpy),
        material=steel,
        name="shoulder_pin",
    )
    for side, y in enumerate((-0.195, 0.195)):
        cap_geom, cap_origin = _cyl_y(0.036, 0.035)
        wall_mount.visual(
            cap_geom,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=cap_origin.rpy),
            material=steel,
            name=f"shoulder_pin_cap_{side}",
        )

    # Raised wall bolts, embedded slightly into the plate face.
    bolt_index = 0
    for y in (-0.190, 0.190):
        for z in (-0.270, 0.0, 0.270):
            bolt_geom, bolt_origin = _cyl_x(0.017, 0.018)
            wall_mount.visual(
                bolt_geom,
                origin=Origin(xyz=(-0.195, y, z), rpy=bolt_origin.rpy),
                material=black,
                name=f"wall_bolt_{bolt_index}",
            )
            bolt_index += 1

    first_link = model.part("first_link")
    hub_geom, hub_origin = _cyl_y(0.086, 0.200)
    first_link.visual(
        hub_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hub_origin.rpy),
        material=link_mat,
        name="shoulder_hub",
    )
    first_link.visual(
        mesh_from_cadquery(
            _tapered_beam(
                length=0.590,
                width_start=0.180,
                width_end=0.130,
                height_start=0.125,
                height_end=0.090,
            ),
            "first_tapered_web",
            tolerance=0.0015,
        ),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=link_mat,
        name="first_tapered_web",
    )
    # Proud side ribs stiffen the tapered link without closing the elbow fork.
    for side, y in enumerate((-0.081, 0.081)):
        first_link.visual(
            Box((0.520, 0.024, 0.026)),
            origin=Origin(xyz=(0.340, y, 0.048)),
            material=cast,
            name=f"first_top_rib_{side}",
        )
        first_link.visual(
            Box((0.440, 0.020, 0.022)),
            origin=Origin(xyz=(0.385, y, -0.045)),
            material=cast,
            name=f"first_lower_rib_{side}",
        )

    first_link.visual(
        Box((0.105, 0.275, 0.092)),
        origin=Origin(xyz=(0.640, 0.0, 0.0)),
        material=cast,
        name="elbow_fork_bridge",
    )
    for side, (y, cheek_name) in enumerate(
        ((-0.126, "elbow_cheek_0"), (0.126, "elbow_cheek_1"))
    ):
        first_link.visual(
            Box((0.225, 0.047, 0.168)),
            origin=Origin(xyz=(0.765, y, 0.0)),
            material=cast,
            name=cheek_name,
        )
        for z in (-0.052, 0.052):
            bolt_geom, bolt_origin = _cyl_y(0.010, 0.012)
            first_link.visual(
                bolt_geom,
                origin=Origin(
                    xyz=(0.765, y + (0.026 if y > 0 else -0.026), z),
                    rpy=bolt_origin.rpy,
                ),
                material=black,
                name=f"elbow_cheek_bolt_{side}_{'top' if z > 0 else 'bottom'}",
            )

    elbow_pin_geom, elbow_pin_origin = _cyl_y(0.019, 0.340)
    first_link.visual(
        elbow_pin_geom,
        origin=Origin(xyz=(0.820, 0.0, 0.0), rpy=elbow_pin_origin.rpy),
        material=steel,
        name="elbow_pin",
    )

    forearm = model.part("forearm")
    elbow_hub_geom, elbow_hub_origin = _cyl_y(0.084, 0.162)
    forearm.visual(
        elbow_hub_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=elbow_hub_origin.rpy),
        material=forearm_mat,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.105, 0.142, 0.122)),
        origin=Origin(xyz=(0.091, 0.0, 0.0)),
        material=forearm_mat,
        name="elbow_split_block",
    )
    forearm.visual(
        mesh_from_cadquery(
            _tapered_beam(
                length=0.390,
                width_start=0.120,
                width_end=0.088,
                height_start=0.092,
                height_end=0.066,
            ),
            "forearm_tapered_web",
            tolerance=0.0015,
        ),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material=forearm_mat,
        name="forearm_tapered_web",
    )
    for side, y in enumerate((-0.058, 0.058)):
        forearm.visual(
            Box((0.330, 0.018, 0.019)),
            origin=Origin(xyz=(0.270, y, 0.036)),
            material=cast,
            name=f"forearm_side_rib_{side}",
        )
    forearm.visual(
        Box((0.095, 0.176, 0.070)),
        origin=Origin(xyz=(0.405, 0.0, 0.0)),
        material=cast,
        name="wrist_fork_bridge",
    )
    for side, (y, cheek_name) in enumerate(
        ((-0.077, "wrist_cheek_0"), (0.077, "wrist_cheek_1"))
    ):
        forearm.visual(
            Box((0.160, 0.037, 0.104)),
            origin=Origin(xyz=(0.505, y, 0.0)),
            material=cast,
            name=cheek_name,
        )

    wrist_pin_geom, wrist_pin_origin = _cyl_y(0.014, 0.210)
    forearm.visual(
        wrist_pin_geom,
        origin=Origin(xyz=(0.520, 0.0, 0.0), rpy=wrist_pin_origin.rpy),
        material=steel,
        name="wrist_pin",
    )

    end_plate = model.part("end_plate")
    wrist_barrel_geom, wrist_barrel_origin = _cyl_y(0.043, 0.090)
    end_plate.visual(
        wrist_barrel_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=wrist_barrel_origin.rpy),
        material=forearm_mat,
        name="wrist_barrel",
    )
    end_plate.visual(
        Box((0.035, 0.040, 0.032)),
        origin=Origin(xyz=(0.057, 0.0, 0.0)),
        material=forearm_mat,
        name="wrist_throat",
    )
    end_plate.visual(
        Box((0.175, 0.070, 0.046)),
        origin=Origin(xyz=(0.1575, 0.0, 0.0)),
        material=forearm_mat,
        name="wrist_neck",
    )
    end_plate.visual(
        Box((0.026, 0.230, 0.230)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=plate_mat,
        name="square_plate",
    )
    end_plate.visual(
        Box((0.022, 0.186, 0.186)),
        origin=Origin(xyz=(0.266, 0.0, 0.0)),
        material=cast,
        name="front_reinforcing_pad",
    )
    for index, (y, z) in enumerate(
        ((-0.072, -0.072), (-0.072, 0.072), (0.072, -0.072), (0.072, 0.072))
    ):
        bolt_geom, bolt_origin = _cyl_x(0.011, 0.012)
        end_plate.visual(
            bolt_geom,
            origin=Origin(xyz=(0.280, y, z), rpy=bolt_origin.rpy),
            material=black,
            name=f"plate_bolt_{index}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=240.0, velocity=0.65, lower=-0.45, upper=0.75),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=forearm,
        origin=Origin(xyz=(0.820, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.8, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=end_plate,
        origin=Origin(xyz=(0.520, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.0, lower=-0.60, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_mount = object_model.get_part("wall_mount")
    first_link = object_model.get_part("first_link")
    forearm = object_model.get_part("forearm")
    end_plate = object_model.get_part("end_plate")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    # Captured shafts intentionally run through rotating hubs.  The overlaps are
    # local and mechanical, and the checks below prove each shaft remains inside
    # the corresponding knuckle with real clearance to the support cheeks.
    ctx.allow_overlap(
        wall_mount,
        first_link,
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        reason="The fixed shoulder shaft is intentionally captured through the rotating shoulder hub.",
    )
    ctx.allow_overlap(
        first_link,
        forearm,
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The elbow shaft is intentionally captured through the deep central elbow knuckle.",
    )
    ctx.allow_overlap(
        forearm,
        end_plate,
        elem_a="wrist_pin",
        elem_b="wrist_barrel",
        reason="The wrist shaft is intentionally captured through the end-plate barrel.",
    )

    ctx.expect_within(
        wall_mount,
        first_link,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_hub",
        margin=0.0,
        name="shoulder pin runs through hub bore",
    )
    ctx.expect_overlap(
        wall_mount,
        first_link,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        min_overlap=0.180,
        name="shoulder pin spans rotating hub",
    )
    ctx.expect_gap(
        wall_mount,
        first_link,
        axis="y",
        positive_elem="shoulder_cheek_1",
        negative_elem="shoulder_hub",
        min_gap=0.020,
        name="positive shoulder cheek clears hub",
    )
    ctx.expect_gap(
        first_link,
        wall_mount,
        axis="y",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_cheek_0",
        min_gap=0.020,
        name="negative shoulder cheek clears hub",
    )

    ctx.expect_within(
        first_link,
        forearm,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_hub",
        margin=0.0,
        name="elbow pin runs through hub bore",
    )
    ctx.expect_gap(
        first_link,
        forearm,
        axis="y",
        positive_elem="elbow_cheek_1",
        negative_elem="elbow_hub",
        min_gap=0.018,
        name="positive elbow cheek clears knuckle",
    )
    ctx.expect_gap(
        forearm,
        first_link,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_cheek_0",
        min_gap=0.018,
        name="negative elbow cheek clears knuckle",
    )

    ctx.expect_within(
        forearm,
        end_plate,
        axes="xz",
        inner_elem="wrist_pin",
        outer_elem="wrist_barrel",
        margin=0.0,
        name="wrist pin runs through barrel",
    )
    ctx.expect_gap(
        forearm,
        end_plate,
        axis="y",
        positive_elem="wrist_cheek_1",
        negative_elem="wrist_barrel",
        min_gap=0.010,
        name="positive wrist cheek clears barrel",
    )
    ctx.expect_gap(
        end_plate,
        forearm,
        axis="y",
        positive_elem="wrist_barrel",
        negative_elem="wrist_cheek_0",
        min_gap=0.010,
        name="negative wrist cheek clears barrel",
    )

    rest_tip = ctx.part_world_position(end_plate)
    with ctx.pose({shoulder: 0.70, elbow: -0.55, wrist: 0.55}):
        raised_tip = ctx.part_world_position(end_plate)
        ctx.expect_gap(
            end_plate,
            forearm,
            axis="x",
            positive_elem="square_plate",
            negative_elem="wrist_cheek_1",
            min_gap=0.015,
            name="raised wrist plate clears fork",
        )
    with ctx.pose({shoulder: -0.40, elbow: 0.65, wrist: -0.55}):
        lowered_tip = ctx.part_world_position(end_plate)
        ctx.expect_gap(
            end_plate,
            forearm,
            axis="x",
            positive_elem="square_plate",
            negative_elem="wrist_cheek_0",
            min_gap=0.015,
            name="lowered wrist plate clears fork",
        )
    ctx.check(
        "shoulder and elbow move the tip through a useful vertical range",
        rest_tip is not None
        and raised_tip is not None
        and lowered_tip is not None
        and raised_tip[2] > rest_tip[2] + 0.18
        and lowered_tip[2] < rest_tip[2] - 0.16,
        details=f"rest={rest_tip}, raised={raised_tip}, lowered={lowered_tip}",
    )

    return ctx.report()


object_model = build_object_model()
