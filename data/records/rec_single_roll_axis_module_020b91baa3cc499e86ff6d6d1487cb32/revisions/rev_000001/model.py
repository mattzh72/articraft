from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_WIDTH = 0.125
PLATE_HEIGHT = 0.190
PLATE_THICKNESS = 0.010
PLATE_CORNER_RADIUS = 0.012

MOUNT_HOLE_RADIUS = 0.0045
MOUNT_HOLE_Y = 0.040
MOUNT_HOLE_Z = 0.062

SUPPORT_WIDTH = 0.038
SUPPORT_FRONT_X = 0.072
BOSS_LENGTH = 0.020
BOSS_RADIUS = 0.026
BORE_RADIUS = 0.0135
BORE_START_X = 0.018
BEARING_RECESS_DEPTH = 0.004
BEARING_RECESS_RADIUS = 0.019

SHAFT_RADIUS = 0.011
SHAFT_FRONT_CLEARANCE = 0.0
SHAFT_FORWARD_LENGTH = 0.012
HUB_RADIUS = 0.022
HUB_LENGTH = 0.014
FLANGE_RADIUS = 0.038
FLANGE_THICKNESS = 0.012
LUG_X = 0.010
LUG_Y = 0.012
LUG_Z = 0.020


def _backplate_and_support_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .rect(PLATE_WIDTH, PLATE_HEIGHT)
        .extrude(PLATE_THICKNESS / 2.0, both=True)
        .edges("|X")
        .fillet(PLATE_CORNER_RADIUS)
    )

    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-MOUNT_HOLE_Y, -MOUNT_HOLE_Z),
                (-MOUNT_HOLE_Y, MOUNT_HOLE_Z),
                (MOUNT_HOLE_Y, -MOUNT_HOLE_Z),
                (MOUNT_HOLE_Y, MOUNT_HOLE_Z),
            ]
        )
        .circle(MOUNT_HOLE_RADIUS)
        .cutThruAll()
    )

    upper_rib_profile = [
        (PLATE_THICKNESS / 2.0, 0.018),
        (0.018, 0.050),
        (0.046, 0.044),
        (SUPPORT_FRONT_X - BOSS_LENGTH, 0.028),
        (SUPPORT_FRONT_X - BOSS_LENGTH, 0.018),
    ]
    upper_rib = (
        cq.Workplane("XZ")
        .polyline(upper_rib_profile)
        .close()
        .extrude(SUPPORT_WIDTH / 2.0, both=True)
    )
    lower_rib = (
        cq.Workplane("XZ")
        .polyline([(x_pos, -z_pos) for x_pos, z_pos in upper_rib_profile])
        .close()
        .extrude(SUPPORT_WIDTH / 2.0, both=True)
    )

    rear_pad = (
        cq.Workplane("YZ")
        .workplane(offset=0.004)
        .circle(0.028)
        .extrude(0.005, both=True)
    )

    support_ring = (
        cq.Workplane("YZ")
        .workplane(offset=SUPPORT_FRONT_X - BOSS_LENGTH / 2.0)
        .circle(BOSS_RADIUS)
        .circle(BORE_RADIUS + 0.002)
        .extrude(BOSS_LENGTH / 2.0, both=True)
    )

    body = plate.union(upper_rib).union(lower_rib).union(rear_pad).union(support_ring)
    return body.combine(clean=True).clean()


def _output_flange_shape() -> cq.Workplane:
    shaft = (
        cq.Workplane("YZ")
        .workplane(offset=SHAFT_FRONT_CLEARANCE)
        .circle(SHAFT_RADIUS)
        .extrude(SHAFT_FORWARD_LENGTH)
    )

    hub = cq.Workplane("YZ").workplane(offset=0.0).circle(HUB_RADIUS).extrude(HUB_LENGTH)
    flange = (
        cq.Workplane("YZ")
        .workplane(offset=0.014)
        .circle(FLANGE_RADIUS)
        .extrude(FLANGE_THICKNESS)
    )

    lug = cq.Workplane("XY").box(LUG_X, LUG_Y, LUG_Z).translate(
        (SHAFT_FORWARD_LENGTH + FLANGE_THICKNESS * 0.55, 0.0, FLANGE_RADIUS - 0.004)
    )

    shape = shaft.union(hub).union(flange).union(lug)

    shape = (
        shape.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, 0.0)])
        .circle(0.012)
        .cutBlind(0.004)
    )

    bolt_circle = 0.024
    shape = (
        shape.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (0.0, bolt_circle),
                (bolt_circle * 0.866, -bolt_circle * 0.5),
                (-bolt_circle * 0.866, -bolt_circle * 0.5),
            ]
        )
        .circle(0.0034)
        .cutBlind(0.008)
    )
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_roll_cartridge")

    wall_gray = model.material("wall_gray", rgba=(0.69, 0.71, 0.73, 1.0))
    zinc = model.material("zinc", rgba=(0.80, 0.82, 0.85, 1.0))
    dark_finish = model.material("dark_finish", rgba=(0.20, 0.22, 0.24, 1.0))

    cartridge_body = model.part("cartridge_body")
    cartridge_body.visual(
        mesh_from_cadquery(_backplate_and_support_shape(), "cartridge_body"),
        material=wall_gray,
        name="cartridge_body_shell",
    )
    cartridge_body.inertial = Inertial.from_geometry(
        Box((SUPPORT_FRONT_X, PLATE_WIDTH, PLATE_HEIGHT)),
        mass=2.8,
        origin=Origin(xyz=(SUPPORT_FRONT_X / 2.0 - 0.002, 0.0, 0.0)),
    )

    output_flange = model.part("output_flange")
    output_flange.visual(
        mesh_from_cadquery(_output_flange_shape(), "output_flange"),
        material=zinc,
        name="output_flange_shell",
    )
    output_flange.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(
            xyz=(SHAFT_FORWARD_LENGTH + FLANGE_THICKNESS * 0.5, 0.0, FLANGE_RADIUS + 0.004),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=dark_finish,
        name="lug_cap",
    )
    output_flange.inertial = Inertial.from_geometry(
        Cylinder(radius=FLANGE_RADIUS, length=SHAFT_FORWARD_LENGTH + FLANGE_THICKNESS),
        mass=0.65,
        origin=Origin(
            xyz=((SHAFT_FORWARD_LENGTH + FLANGE_THICKNESS) / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
    )

    model.articulation(
        "flange_spin",
        ArticulationType.REVOLUTE,
        parent=cartridge_body,
        child=output_flange,
        origin=Origin(xyz=(SUPPORT_FRONT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-2.8,
            upper=2.8,
            effort=8.0,
            velocity=6.0,
        ),
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

    cartridge_body = object_model.get_part("cartridge_body")
    output_flange = object_model.get_part("output_flange")
    flange_spin = object_model.get_articulation("flange_spin")

    ctx.check(
        "flange spin uses supported longitudinal axis",
        flange_spin.axis == (1.0, 0.0, 0.0),
        details=f"axis={flange_spin.axis}",
    )

    with ctx.pose({flange_spin: 0.0}):
        ctx.expect_origin_gap(
            output_flange,
            cartridge_body,
            axis="x",
            min_gap=0.068,
            max_gap=0.076,
            name="flange axis sits forward of the backplate",
        )
        ctx.expect_overlap(
            output_flange,
            cartridge_body,
            axes="yz",
            min_overlap=0.060,
            name="flange stays centered on the cartridge support footprint",
        )
        rest_aabb = ctx.part_world_aabb(output_flange)

    with ctx.pose({flange_spin: pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(output_flange)

    rest_ok = rest_aabb is not None
    turned_ok = turned_aabb is not None
    silhouette_rotates = False
    details = f"rest={rest_aabb}, turned={turned_aabb}"
    if rest_ok and turned_ok:
        rest_min, rest_max = rest_aabb
        turned_min, turned_max = turned_aabb
        silhouette_rotates = (
            abs((turned_max[0] - turned_min[0]) - (rest_max[0] - rest_min[0])) < 0.003
            and (
                turned_min[1] < rest_min[1] - 0.006
                or turned_max[1] > rest_max[1] + 0.006
            )
            and (
                turned_max[2] < rest_max[2] - 0.006
                or turned_min[2] > rest_min[2] + 0.006
            )
        )
    ctx.check(
        "indexed flange silhouette rotates about x",
        silhouette_rotates,
        details=details,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
