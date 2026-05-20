from __future__ import annotations

import math

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
    mesh_from_geometry,
)


def _hollow_tube_geometry(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    """Open-ended cylindrical launch tube, authored along local +X."""
    geom = MeshGeometry()
    x0 = 0.0
    x1 = length
    outer_front: list[int] = []
    outer_back: list[int] = []
    inner_front: list[int] = []
    inner_back: list[int] = []

    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        outer_front.append(geom.add_vertex(x1, outer_radius * c, outer_radius * s))
        outer_back.append(geom.add_vertex(x0, outer_radius * c, outer_radius * s))
        inner_front.append(geom.add_vertex(x1, inner_radius * c, inner_radius * s))
        inner_back.append(geom.add_vertex(x0, inner_radius * c, inner_radius * s))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer skin.
        geom.add_face(outer_back[i], outer_back[j], outer_front[j])
        geom.add_face(outer_back[i], outer_front[j], outer_front[i])
        # Inner bore, wound inward.
        geom.add_face(inner_back[j], inner_back[i], inner_front[i])
        geom.add_face(inner_front[j], inner_back[j], inner_front[i])
        # Front annular lip.
        geom.add_face(outer_front[i], outer_front[j], inner_front[j])
        geom.add_face(outer_front[i], inner_front[j], inner_front[i])
        # Rear annular lip.
        geom.add_face(outer_back[j], outer_back[i], inner_back[i])
        geom.add_face(inner_back[j], outer_back[j], inner_back[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="towed_twin_tube_missile_launcher")

    olive = model.material("olive_drab", rgba=(0.28, 0.34, 0.22, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.13, 0.17, 0.11, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.02, 0.022, 0.018, 1.0))
    steel = model.material("dark_gunmetal", rgba=(0.20, 0.21, 0.20, 1.0))
    bore = model.material("tube_bore_black", rgba=(0.005, 0.006, 0.005, 1.0))
    yellow = model.material("safety_yellow", rgba=(0.82, 0.63, 0.13, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((1.35, 0.62, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=olive,
        name="deck",
    )
    chassis.visual(
        Box((0.92, 0.12, 0.08)),
        origin=Origin(xyz=(-1.04, 0.0, 0.49)),
        material=olive,
        name="tow_bar",
    )
    chassis.visual(
        Cylinder(radius=0.085, length=0.06),
        origin=Origin(xyz=(-1.52, 0.0, 0.49), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tow_eye",
    )
    chassis.visual(
        Cylinder(radius=0.038, length=1.18),
        origin=Origin(xyz=(0.23, 0.0, 0.34), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    for suffix, y in (("0", 0.62), ("1", -0.62)):
        chassis.visual(
            Cylinder(radius=0.28, length=0.16),
            origin=Origin(xyz=(0.23, y, 0.34), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"wheel_{suffix}",
        )
        chassis.visual(
            Cylinder(radius=0.105, length=0.18),
            origin=Origin(xyz=(0.23, y, 0.34), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"wheel_hub_{suffix}",
        )
        chassis.visual(
            Box((0.11, 0.10, 0.23)),
            origin=Origin(xyz=(0.23, 0.27 if y > 0 else -0.27, 0.43)),
            material=dark_olive,
            name=f"axle_hanger_{suffix}",
        )

    # Side hinge brackets keep the folding stabilizers visibly clipped to the chassis.
    for suffix, y_sign in (("0", 1.0), ("1", -1.0)):
        y = y_sign * 0.46
        chassis.visual(
            Box((0.16, 0.08, 0.28)),
            origin=Origin(xyz=(-0.45, y_sign * 0.34, 0.49)),
            material=dark_olive,
            name=f"hinge_web_{suffix}",
        )
        chassis.visual(
            Box((0.12, 0.08, 0.06)),
            origin=Origin(xyz=(-0.45, y_sign * 0.42, 0.405)),
            material=dark_olive,
            name=f"hinge_boss_{suffix}",
        )
        chassis.visual(
            Cylinder(radius=0.028, length=0.23),
            origin=Origin(xyz=(-0.45, y, 0.52)),
            material=steel,
            name=f"hinge_pin_{suffix}",
        )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.30, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=dark_olive,
        name="turntable",
    )
    turret.visual(
        Cylinder(radius=0.135, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=olive,
        name="pedestal",
    )
    turret.visual(
        Box((0.34, 0.88, 0.17)),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=olive,
        name="yoke_bridge",
    )
    for suffix, y in (("0", 0.44), ("1", -0.44)):
        turret.visual(
            Box((0.20, 0.08, 0.58)),
            origin=Origin(xyz=(0.0, y, 0.76)),
            material=olive,
            name=f"yoke_cheek_{suffix}",
        )
        turret.visual(
            Cylinder(radius=0.065, length=0.085),
            origin=Origin(xyz=(0.0, y, 0.76), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"trunnion_bushing_{suffix}",
        )

    model.articulation(
        "chassis_to_turret",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.6, lower=-math.pi, upper=math.pi),
    )

    cradle = model.part("cradle")
    tube_mesh = _hollow_tube_geometry(0.13, 0.092, 1.56)
    for suffix, y in (("0", 0.18), ("1", -0.18)):
        cradle.visual(
            mesh_from_geometry(tube_mesh, f"launch_tube_{suffix}"),
            origin=Origin(xyz=(0.03, y, 0.09)),
            material=olive,
            name=f"tube_{suffix}",
        )
        cradle.visual(
            Cylinder(radius=0.094, length=0.012),
            origin=Origin(xyz=(1.54, y, 0.09), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bore,
            name=f"dark_bore_{suffix}",
        )
    cradle.visual(
        Box((0.16, 0.62, 0.22)),
        origin=Origin(xyz=(0.08, 0.0, 0.05)),
        material=dark_olive,
        name="rear_clamp",
    )
    cradle.visual(
        Box((0.10, 0.58, 0.10)),
        origin=Origin(xyz=(1.12, 0.0, -0.035)),
        material=dark_olive,
        name="front_saddle",
    )
    cradle.visual(
        Cylinder(radius=0.052, length=0.80),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elevation_shaft",
    )
    cradle.visual(
        Box((0.44, 0.12, 0.05)),
        origin=Origin(xyz=(0.34, 0.0, -0.045)),
        material=yellow,
        name="balance_bar",
    )

    model.articulation(
        "turret_to_cradle",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.5, lower=0.0, upper=1.10),
    )

    for suffix, y_sign, axis_z in (("0", 1.0, 1.0), ("1", -1.0, -1.0)):
        outrigger = model.part(f"outrigger_{suffix}")
        outrigger.visual(
            Cylinder(radius=0.052, length=0.16),
            origin=Origin(),
            material=steel,
            name="hinge_sleeve",
        )
        outrigger.visual(
            Box((0.88, 0.07, 0.07)),
            origin=Origin(xyz=(0.49, 0.0, -0.045)),
            material=olive,
            name="folding_arm",
        )
        outrigger.visual(
            Cylinder(radius=0.035, length=0.38),
            origin=Origin(xyz=(0.86, 0.0, -0.27)),
            material=steel,
            name="drop_leg",
        )
        outrigger.visual(
            Box((0.25, 0.18, 0.04)),
            origin=Origin(xyz=(0.86, 0.0, -0.48)),
            material=dark_olive,
            name="foot_pad",
        )
        model.articulation(
            f"chassis_to_outrigger_{suffix}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=outrigger,
            origin=Origin(xyz=(-0.45, y_sign * 0.46, 0.52)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=90.0, velocity=0.8, lower=0.0, upper=1.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    turret = object_model.get_part("turret")
    cradle = object_model.get_part("cradle")
    outrigger_0 = object_model.get_part("outrigger_0")
    outrigger_1 = object_model.get_part("outrigger_1")
    yaw = object_model.get_articulation("chassis_to_turret")
    elevation = object_model.get_articulation("turret_to_cradle")
    stabilizer_0 = object_model.get_articulation("chassis_to_outrigger_0")
    stabilizer_1 = object_model.get_articulation("chassis_to_outrigger_1")

    for suffix, outrigger in (("0", outrigger_0), ("1", outrigger_1)):
        ctx.allow_overlap(
            chassis,
            outrigger,
            elem_a=f"hinge_pin_{suffix}",
            elem_b="hinge_sleeve",
            reason="The stabilizer sleeve is intentionally captured around the side hinge pin so the folding leg stays clipped to the chassis.",
        )
        ctx.expect_overlap(
            chassis,
            outrigger,
            axes="xy",
            elem_a=f"hinge_pin_{suffix}",
            elem_b="hinge_sleeve",
            min_overlap=0.040,
            name=f"outrigger_{suffix} sleeve surrounds side hinge in plan",
        )
        ctx.expect_overlap(
            chassis,
            outrigger,
            axes="z",
            elem_a=f"hinge_pin_{suffix}",
            elem_b="hinge_sleeve",
            min_overlap=0.12,
            name=f"outrigger_{suffix} sleeve remains on hinge height",
        )

    ctx.expect_contact(
        turret,
        chassis,
        elem_a="turntable",
        elem_b="deck",
        contact_tol=0.002,
        name="azimuth turntable sits on chassis deck",
    )
    ctx.expect_contact(
        cradle,
        turret,
        elem_a="elevation_shaft",
        elem_b="trunnion_bushing_0",
        contact_tol=0.004,
        name="elevation shaft reaches first yoke bushing",
    )
    ctx.expect_contact(
        cradle,
        turret,
        elem_a="elevation_shaft",
        elem_b="trunnion_bushing_1",
        contact_tol=0.004,
        name="elevation shaft reaches second yoke bushing",
    )

    tube0_aabb = ctx.part_element_world_aabb(cradle, elem="tube_0")
    tube1_aabb = ctx.part_element_world_aabb(cradle, elem="tube_1")
    if tube0_aabb is not None and tube1_aabb is not None:
        tube0_center_y = 0.5 * (tube0_aabb[0][1] + tube0_aabb[1][1])
        tube1_center_y = 0.5 * (tube1_aabb[0][1] + tube1_aabb[1][1])
        ctx.check(
            "twin tubes are separated side-by-side",
            tube0_center_y > 0.12 and tube1_center_y < -0.12,
            details=f"tube centers y=({tube0_center_y:.3f}, {tube1_center_y:.3f})",
        )
    else:
        ctx.fail("twin tube aabb available", "Could not compute both tube element AABBs.")

    rest_tube_aabb = ctx.part_element_world_aabb(cradle, elem="tube_0")
    with ctx.pose({elevation: 1.0}):
        raised_tube_aabb = ctx.part_element_world_aabb(cradle, elem="tube_0")
    ctx.check(
        "elevation joint raises launcher tubes",
        rest_tube_aabb is not None
        and raised_tube_aabb is not None
        and raised_tube_aabb[1][2] > rest_tube_aabb[1][2] + 0.35,
        details=f"rest={rest_tube_aabb}, raised={raised_tube_aabb}",
    )

    rest_cradle_aabb = ctx.part_world_aabb(cradle)
    with ctx.pose({yaw: 0.9}):
        yawed_cradle_aabb = ctx.part_world_aabb(cradle)
    if rest_cradle_aabb is not None and yawed_cradle_aabb is not None:
        rest_center_y = 0.5 * (rest_cradle_aabb[0][1] + rest_cradle_aabb[1][1])
        yawed_center_y = 0.5 * (yawed_cradle_aabb[0][1] + yawed_cradle_aabb[1][1])
        ctx.check(
            "azimuth joint slews launcher head about vertical axis",
            yawed_center_y > rest_center_y + 0.45,
            details=f"rest_y={rest_center_y:.3f}, yawed_y={yawed_center_y:.3f}",
        )
    else:
        ctx.fail("cradle aabb available for yaw proof", "Could not compute cradle AABBs.")

    for suffix, joint, direction in (
        ("0", stabilizer_0, 1.0),
        ("1", stabilizer_1, -1.0),
    ):
        outrigger = object_model.get_part(f"outrigger_{suffix}")
        rest_pad = ctx.part_element_world_aabb(outrigger, elem="foot_pad")
        with ctx.pose({joint: 1.35}):
            deployed_pad = ctx.part_element_world_aabb(outrigger, elem="foot_pad")
        if rest_pad is not None and deployed_pad is not None:
            rest_y = 0.5 * (rest_pad[0][1] + rest_pad[1][1])
            deployed_y = 0.5 * (deployed_pad[0][1] + deployed_pad[1][1])
            ctx.check(
                f"outrigger_{suffix} folds outward from side hinge",
                direction * deployed_y > direction * rest_y + 0.45,
                details=f"rest_y={rest_y:.3f}, deployed_y={deployed_y:.3f}",
            )
        else:
            ctx.fail(f"outrigger_{suffix} foot pad aabb available", "Could not compute foot pad AABBs.")

    return ctx.report()


object_model = build_object_model()
