from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_L = 1.80
BODY_W = 0.82
BODY_H = 0.70
BODY_BOTTOM_Z = 0.22
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_H
WALL_T = 0.055


def _freezer_tub_shape() -> cq.Workplane:
    """One-piece open insulated freezer shell with a visible deep cavity."""
    outer = (
        cq.Workplane("XY")
        .box(BODY_L, BODY_W, BODY_H, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_BOTTOM_Z))
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            BODY_L - 2.0 * WALL_T,
            BODY_W - 2.0 * WALL_T,
            BODY_H + 0.04,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_BOTTOM_Z + WALL_T))
    )
    shell = outer.cut(cavity)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deep_island_freezer")

    enamel = model.material("warm_white_enamel", color=(0.88, 0.91, 0.91, 1.0))
    liner = model.material("pale_cavity_liner", color=(0.72, 0.82, 0.90, 1.0))
    rail_metal = model.material("brushed_aluminum", color=(0.58, 0.60, 0.60, 1.0))
    dark_rubber = model.material("dark_rubber", color=(0.03, 0.03, 0.03, 1.0))
    black = model.material("black_tire", color=(0.005, 0.005, 0.005, 1.0))
    steel = model.material("zinc_steel", color=(0.46, 0.48, 0.49, 1.0))
    lid_white = model.material("lid_white", color=(0.94, 0.96, 0.96, 1.0))
    grip_gray = model.material("recessed_grip_gray", color=(0.18, 0.20, 0.21, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_freezer_tub_shape(), "open_freezer_tub", tolerance=0.003),
        material=enamel,
        name="tub_shell",
    )

    # A blue-grey visible bottom liner makes the body read as a deep hollow freezer.
    body.visual(
        Box((BODY_L - 2.0 * WALL_T - 0.02, BODY_W - 2.0 * WALL_T - 0.02, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + WALL_T + 0.009)),
        material=liner,
        name="cavity_floor",
    )

    # Tall side rails carry three stepped tracks for the narrow sliding lids.
    side_rail_h = 0.16
    side_rail_y = BODY_W / 2.0 - 0.030
    side_rail_z = BODY_TOP_Z + side_rail_h / 2.0
    for sign, side in ((1.0, "front"), (-1.0, "rear")):
        body.visual(
            Box((BODY_L + 0.02, 0.060, side_rail_h)),
            origin=Origin(xyz=(0.0, sign * side_rail_y, side_rail_z)),
            material=rail_metal,
            name=f"{side}_rail",
        )

    lane_tops = [BODY_TOP_Z + 0.035, BODY_TOP_Z + 0.070, BODY_TOP_Z + 0.105]
    ledge_y = BODY_W / 2.0 - 0.075
    for lane, top_z in enumerate(lane_tops):
        for sign, side in ((1.0, "front"), (-1.0, "rear")):
            body.visual(
                Box((BODY_L - 0.08, 0.035, 0.012)),
                origin=Origin(xyz=(0.0, sign * ledge_y, top_z - 0.006)),
                material=rail_metal,
                name=f"track_{lane}_{side}",
            )

    # Thick rubber bumpers wrap and protect each vertical corner.
    for ix, xsign in enumerate((-1.0, 1.0)):
        for iy, ysign in enumerate((-1.0, 1.0)):
            body.visual(
                Box((0.090, 0.090, 0.300)),
                origin=Origin(
                    xyz=(
                        xsign * (BODY_L / 2.0 + 0.014),
                        ysign * (BODY_W / 2.0 + 0.014),
                        BODY_BOTTOM_Z + 0.230,
                    )
                ),
                material=dark_rubber,
                name=f"corner_bumper_{ix}_{iy}",
            )

    # Three insulated lids.  Each has runners captured in a different rail step so
    # the panels can pass over one another when slid open.
    lid_len = 0.540
    lid_w = 0.635
    lid_th = 0.025
    runner_h = 0.010
    runner_w = 0.018
    runner_y = BODY_W / 2.0 - 0.085
    lid_centers_x = (-0.580, 0.0, 0.580)
    for i, (x0, lane_top_z) in enumerate(zip(lid_centers_x, lane_tops)):
        lid = model.part(f"lid_{i}")
        panel_center_z = lane_top_z + runner_h + lid_th / 2.0
        joint = model.articulation(
            f"body_to_lid_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=lid,
            origin=Origin(xyz=(x0, 0.0, panel_center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=-0.22, upper=0.22),
        )
        joint.meta["qc_samples"] = [0.0]

        lid.visual(
            Box((lid_len, lid_w, lid_th)),
            origin=Origin(),
            material=lid_white,
            name="panel",
        )
        for sign, side in ((1.0, "front"), (-1.0, "rear")):
            lid.visual(
                Box((lid_len - 0.030, runner_w, runner_h)),
                origin=Origin(xyz=(0.0, sign * runner_y, -(lid_th + runner_h) / 2.0)),
                material=rail_metal,
                name=f"runner_{side}",
            )
        # Low recessed pull trough on each lid, attached to the top face.
        lid.visual(
            Box((0.200, 0.035, 0.009)),
            origin=Origin(xyz=(0.0, -0.120, lid_th / 2.0 + 0.0045)),
            material=grip_gray,
            name="pull_grip",
        )

    tire_meshes = []
    wheel_meshes = []
    for i in range(4):
        tire_geom = TireGeometry(
            0.055,
            0.042,
            inner_radius=0.038,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.004, count=14, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        )
        wheel_geom = WheelGeometry(
            0.037,
            0.044,
            rim=WheelRim(inner_radius=0.026, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.016,
                width=0.032,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.020, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.020),
        )
        tire_meshes.append(mesh_from_geometry(tire_geom, f"caster_tire_{i}"))
        wheel_meshes.append(mesh_from_geometry(wheel_geom, f"caster_rim_{i}"))

    caster_x = BODY_L / 2.0 - 0.150
    caster_y = BODY_W / 2.0 - 0.120
    axle_z = -0.145
    for i, (xsign, ysign) in enumerate(((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))):
        caster = model.part(f"caster_{i}")
        model.articulation(
            f"body_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=caster,
            origin=Origin(xyz=(xsign * caster_x, ysign * caster_y, BODY_BOTTOM_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=4.0),
        )
        caster.visual(
            Box((0.120, 0.120, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=steel,
            name="mount_plate",
        )
        caster.visual(
            Cylinder(radius=0.017, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, -0.050)),
            material=steel,
            name="swivel_stem",
        )
        caster.visual(
            Box((0.110, 0.040, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, -0.070)),
            material=steel,
            name="fork_bridge",
        )
        for sign, side in ((1.0, "outer"), (-1.0, "inner")):
            caster.visual(
                Box((0.012, 0.052, 0.110)),
                origin=Origin(xyz=(sign * 0.036, 0.0, -0.128)),
                material=steel,
                name=f"fork_{side}",
            )
        caster.visual(
            Cylinder(radius=0.010, length=0.092),
            origin=Origin(xyz=(0.0, 0.0, axle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="axle",
        )

        wheel = model.part(f"wheel_{i}")
        model.articulation(
            f"caster_{i}_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, axle_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=30.0),
        )
        wheel.visual(tire_meshes[i], material=black, name="tire")
        wheel.visual(wheel_meshes[i], material=steel, name="rim")

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    for i in range(3):
        lid = object_model.get_part(f"lid_{i}")
        joint = object_model.get_articulation(f"body_to_lid_{i}")
        side = "front"
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=f"runner_{side}",
            negative_elem=f"track_{i}_{side}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"lid_{i} runner rests on its rail",
        )
        ctx.expect_within(
            lid,
            body,
            axes="xy",
            margin=0.020,
            inner_elem="panel",
            outer_elem="tub_shell",
            name=f"lid_{i} spans the top opening",
        )
        rest = ctx.part_world_position(lid)
        with ctx.pose({joint: 0.18}):
            moved = ctx.part_world_position(lid)
        ctx.check(
            f"lid_{i} slides along rails",
            rest is not None and moved is not None and moved[0] > rest[0] + 0.15,
            details=f"rest={rest}, moved={moved}",
        )

    for i in range(4):
        caster = object_model.get_part(f"caster_{i}")
        wheel = object_model.get_part(f"wheel_{i}")
        swivel = object_model.get_articulation(f"body_to_caster_{i}")
        spin = object_model.get_articulation(f"caster_{i}_to_wheel_{i}")
        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="axle",
            elem_b="rim",
            reason="The caster axle is intentionally captured through the wheel hub bore.",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="x",
            elem_a="axle",
            elem_b="rim",
            min_overlap=0.035,
            name=f"caster_{i} axle passes through rim",
        )
        ctx.expect_gap(
            body,
            caster,
            axis="z",
            positive_elem="tub_shell",
            negative_elem="mount_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"caster_{i} bolted under cabinet",
        )
        wheel_rest = ctx.part_world_position(wheel)
        with ctx.pose({swivel: 0.6, spin: 1.2}):
            wheel_moved = ctx.part_world_position(wheel)
        ctx.check(
            f"caster_{i} swivels without detaching wheel",
            wheel_rest is not None
            and wheel_moved is not None
            and abs(wheel_moved[2] - wheel_rest[2]) < 0.002,
            details=f"rest={wheel_rest}, moved={wheel_moved}",
        )

    return ctx.report()


object_model = build_object_model()
