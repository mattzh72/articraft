from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOUSING_SIZE = (0.092, 0.092, 0.040)
POCKET_SIZE = (0.072, 0.052)
POCKET_DEPTH = 0.024
GIMBAL_CENTER_Z = 0.033


def solid_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length * 0.5))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length * 0.5))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def make_housing() -> cq.Workplane:
    width, depth, height = HOUSING_SIZE
    pocket_w, pocket_d = POCKET_SIZE

    body = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((0.0, 0.0, height * 0.5))
        .edges("|Z")
        .fillet(0.005)
        .faces(">Z")
        .chamfer(0.0015)
    )

    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(pocket_w, pocket_d)
        .cutBlind(-POCKET_DEPTH)
    )

    front_pad = solid_box((0.024, 0.004, 0.012), (0.0, 0.025, GIMBAL_CENTER_Z))
    rear_pad = solid_box((0.024, 0.004, 0.012), (0.0, -0.025, GIMBAL_CENTER_Z))

    fused = body.findSolid().fuse(front_pad.val()).fuse(rear_pad.val())
    return cq.Workplane(obj=fused)


def make_outer_gimbal() -> cq.Workplane:
    left_plate = solid_box((0.006, 0.030, 0.018), (-0.012, 0.0, -0.003))
    right_plate = solid_box((0.006, 0.030, 0.018), (0.012, 0.0, -0.003))
    bottom_bridge = solid_box((0.030, 0.010, 0.004), (0.0, 0.0, -0.010))
    front_crossbar = solid_box((0.024, 0.004, 0.004), (0.0, 0.013, 0.0))
    rear_crossbar = solid_box((0.024, 0.004, 0.004), (0.0, -0.013, 0.0))
    front_trunnion = cylinder_y(0.0035, 0.008, (0.0, 0.019, 0.0))
    rear_trunnion = cylinder_y(0.0035, 0.008, (0.0, -0.019, 0.0))

    fused = (
        left_plate.findSolid()
        .fuse(right_plate.val())
        .fuse(bottom_bridge.val())
        .fuse(front_crossbar.val())
        .fuse(rear_crossbar.val())
        .fuse(front_trunnion.val())
        .fuse(rear_trunnion.val())
    )
    return cq.Workplane(obj=fused)


def make_inner_gimbal() -> cq.Workplane:
    carrier = solid_box((0.006, 0.012, 0.010), (0.0, 0.0, -0.001))
    lower_rib = solid_box((0.004, 0.016, 0.004), (0.0, 0.0, -0.005))
    hub = cq.Workplane("XY").circle(0.006).extrude(0.005).translate((0.0, 0.0, 0.0))
    right_trunnion = cylinder_x(0.0025, 0.006, (0.006, 0.0, 0.0))
    left_trunnion = cylinder_x(0.0025, 0.006, (-0.006, 0.0, 0.0))

    fused = (
        carrier.findSolid()
        .fuse(lower_rib.val())
        .fuse(hub.val())
        .fuse(right_trunnion.val())
        .fuse(left_trunnion.val())
    )
    return cq.Workplane(obj=fused)


def make_lever() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(0.0075).extrude(0.0065, taper=-18.0).translate((0.0, 0.0, 0.005))
    shaft = cq.Workplane("XY").circle(0.0045).extrude(0.038).translate((0.0, 0.0, 0.0115))
    tip = cq.Workplane("XY").sphere(0.006).translate((0.0, 0.0, 0.0545))
    fused = collar.findSolid().fuse(shaft.val()).fuse(tip.val())
    return cq.Workplane(obj=fused)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_joystick_module")

    housing_mat = model.material("housing_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    gimbal_mat = model.material("gimbal_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    lever_mat = model.material("lever_black", rgba=(0.10, 0.10, 0.11, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(make_housing(), "housing_shell"),
        material=housing_mat,
        name="housing_shell",
    )

    outer_gimbal = model.part("outer_gimbal")
    outer_gimbal.visual(
        mesh_from_cadquery(make_outer_gimbal(), "outer_gimbal"),
        material=gimbal_mat,
        name="outer_gimbal",
    )

    inner_gimbal = model.part("inner_gimbal")
    inner_gimbal.visual(
        mesh_from_cadquery(make_inner_gimbal(), "inner_gimbal"),
        material=gimbal_mat,
        name="inner_gimbal",
    )

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(make_lever(), "lever_body"),
        material=lever_mat,
        name="lever_body",
    )

    model.articulation(
        "housing_to_outer_gimbal",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=outer_gimbal,
        origin=Origin(xyz=(0.0, 0.0, GIMBAL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "outer_to_inner_gimbal",
        ArticulationType.REVOLUTE,
        parent=outer_gimbal,
        child=inner_gimbal,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "inner_gimbal_to_lever",
        ArticulationType.FIXED,
        parent=inner_gimbal,
        child=lever,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    outer_gimbal = object_model.get_part("outer_gimbal")
    inner_gimbal = object_model.get_part("inner_gimbal")
    lever = object_model.get_part("lever")

    outer_joint = object_model.get_articulation("housing_to_outer_gimbal")
    inner_joint = object_model.get_articulation("outer_to_inner_gimbal")

    ctx.check(
        "two orthogonal revolute gimbal joints",
        outer_joint.articulation_type == ArticulationType.REVOLUTE
        and inner_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(sum(a * b for a, b in zip(outer_joint.axis, inner_joint.axis))) < 1e-9,
        details=f"outer_axis={outer_joint.axis}, inner_axis={inner_joint.axis}",
    )

    ctx.allow_overlap(
        housing,
        outer_gimbal,
        reason="The housing support pads stand in for coaxial bearing pockets around the outer cardan trunnions.",
    )
    ctx.allow_overlap(
        outer_gimbal,
        inner_gimbal,
        reason="The outer yoke and inner carrier use simplified nested bearing seats at the orthogonal cardan axis.",
    )

    ctx.expect_contact(
        outer_gimbal,
        housing,
        name="outer gimbal journals seat against housing supports",
    )
    ctx.expect_contact(
        inner_gimbal,
        outer_gimbal,
        name="inner gimbal journals seat against outer gimbal supports",
    )
    ctx.expect_contact(
        lever,
        inner_gimbal,
        name="lever collar seats on the inner hub",
    )

    def aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    rest_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({outer_joint: 0.25, inner_joint: -0.25}):
        posed_aabb = ctx.part_world_aabb(lever)

    moved = (
        rest_aabb is not None
        and posed_aabb is not None
        and abs(aabb_center(posed_aabb)[0] - aabb_center(rest_aabb)[0]) > 0.005
        and abs(aabb_center(posed_aabb)[1] - aabb_center(rest_aabb)[1]) > 0.005
    )
    ctx.check(
        "lever tip sweeps laterally when both gimbal axes are posed",
        moved,
        details=f"rest={rest_aabb}, posed={posed_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
