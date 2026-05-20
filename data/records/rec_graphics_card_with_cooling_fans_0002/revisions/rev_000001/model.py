from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
import cadquery as cq

BODY_LEN = 0.288
BODY_HEIGHT = 0.118
BODY_THICK = 0.052
FRONT_SHELL_T = 0.020
BACKPLATE_T = 0.004
FRONT_SHELL_Y = BODY_THICK / 2.0 - FRONT_SHELL_T / 2.0
BACKPLATE_Y = -BODY_THICK / 2.0 + BACKPLATE_T / 2.0

FAN_XS = (-0.086, 0.0, 0.086)
FAN_RECESS_R = 0.041
FAN_OPEN_R = 0.036
FAN_ROTOR_R = 0.033
FAN_HUB_R = 0.0105
FAN_DEPTH = 0.006
FAN_CENTER_Y = 0.013

BRACKET_INSERT_X = -BODY_LEN / 2.0 + 0.001


def _make_shroud_shape():
    front_shell = (
        cq.Workplane("XY")
        .box(BODY_LEN, FRONT_SHELL_T, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.005)
        .translate((0.0, FRONT_SHELL_Y, 0.0))
    )
    front_shell = (
        front_shell.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(x, 0.0) for x in FAN_XS])
        .circle(FAN_RECESS_R)
        .cutBlind(-0.0035)
    )
    front_shell = (
        front_shell.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(x, 0.0) for x in FAN_XS])
        .circle(FAN_OPEN_R)
        .cutThruAll()
    )

    top_rail = (
        cq.Workplane("XY")
        .box(BODY_LEN - 0.010, BODY_THICK - 0.006, 0.018)
        .translate((0.0, -0.001, BODY_HEIGHT / 2.0 - 0.009))
    )
    bottom_rail = (
        cq.Workplane("XY")
        .box(BODY_LEN - 0.008, BODY_THICK - 0.002, 0.015)
        .translate((0.0, -0.001, -BODY_HEIGHT / 2.0 + 0.0075))
    )
    left_end = (
        cq.Workplane("XY")
        .box(0.018, BODY_THICK - 0.004, BODY_HEIGHT - 0.010)
        .translate((-BODY_LEN / 2.0 + 0.009, -0.001, 0.0))
    )
    right_end = (
        cq.Workplane("XY")
        .box(0.022, BODY_THICK - 0.006, BODY_HEIGHT - 0.014)
        .translate((BODY_LEN / 2.0 - 0.011, -0.001, 0.0))
    )
    power_connector = (
        cq.Workplane("XY")
        .box(0.032, 0.010, 0.012)
        .translate((0.096, -0.003, BODY_HEIGHT / 2.0 + 0.001))
    )
    lower_splitter = (
        cq.Workplane("XY")
        .box(BODY_LEN - 0.060, 0.018, 0.010)
        .translate((0.010, 0.003, -BODY_HEIGHT / 2.0 + 0.018))
    )

    shroud = (
        front_shell.union(top_rail)
        .union(bottom_rail)
        .union(left_end)
        .union(right_end)
        .union(power_connector)
        .union(lower_splitter)
    )
    return shroud


def _make_backplate_shape():
    plate = (
        cq.Workplane("XY")
        .box(BODY_LEN - 0.004, BACKPLATE_T, BODY_HEIGHT - 0.004)
        .translate((0.0, BACKPLATE_Y, 0.0))
    )
    main_rib = (
        cq.Workplane("XY")
        .box(0.194, BACKPLATE_T * 1.6, 0.022)
        .translate((-0.008, BACKPLATE_Y + 0.0006, 0.020))
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(0.110, BACKPLATE_T * 1.5, 0.012)
        .translate((0.050, BACKPLATE_Y + 0.0005, -0.030))
    )
    tail_rib = (
        cq.Workplane("XY")
        .box(0.050, BACKPLATE_T * 1.4, 0.020)
        .translate((BODY_LEN / 2.0 - 0.040, BACKPLATE_Y + 0.0005, 0.008))
    )
    return plate.union(main_rib).union(lower_rib).union(tail_rib)


def _make_heatsink_shape():
    fin_length = 0.256
    fin_depth = 0.030
    fin_thickness = 0.0011

    core = cq.Workplane("XY").box(0.186, 0.032, 0.026).translate((0.000, -0.008, 0.0))
    left_manifold = (
        cq.Workplane("XY")
        .box(0.012, fin_depth, BODY_HEIGHT - 0.030)
        .translate((-0.128, -0.008, 0.0))
    )
    right_manifold = (
        cq.Workplane("XY")
        .box(0.012, fin_depth, BODY_HEIGHT - 0.030)
        .translate((0.128, -0.008, 0.0))
    )

    heatsink = core.union(left_manifold).union(right_manifold)
    for step in range(-8, 9):
        fin = (
            cq.Workplane("XY")
            .box(fin_length, fin_depth, fin_thickness)
            .translate((0.0, -0.008, step * 0.005))
        )
        heatsink = heatsink.union(fin)
    return heatsink


def _make_fan_rotor_shape():
    tip_ring = (
        cq.Workplane("XZ")
        .circle(FAN_ROTOR_R)
        .circle(FAN_ROTOR_R - 0.0015)
        .extrude(FAN_DEPTH * 0.25, both=True)
    )
    hub = cq.Workplane("XZ").circle(FAN_HUB_R).extrude(FAN_DEPTH * 0.9, both=True)
    cap = cq.Workplane("XZ").circle(FAN_HUB_R * 0.58).extrude(FAN_DEPTH, both=True)

    blade = (
        cq.Workplane("XZ")
        .polyline(
            [
                (FAN_HUB_R * 1.00, -0.004),
                (FAN_ROTOR_R * 0.60, -0.001),
                (FAN_ROTOR_R * 0.96, 0.012),
                (FAN_ROTOR_R * 0.74, 0.021),
                (FAN_HUB_R * 1.18, 0.016),
            ]
        )
        .close()
        .extrude(FAN_DEPTH * 0.36, both=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 18.0)
    )

    rotor = tip_ring.union(hub).union(cap)
    for i in range(9):
        rotor = rotor.union(blade.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), i * 40.0))
    return rotor


def _make_bracket_shape():
    plate = cq.Workplane("XY").box(0.008, 0.019, 0.120).translate((-0.003, 0.0, 0.0))
    top_ear = cq.Workplane("XY").box(0.008, 0.006, 0.012).translate((-0.003, 0.0065, 0.056))
    lower_hook = cq.Workplane("XY").box(0.004, 0.006, 0.008).translate((-0.0055, -0.006, -0.053))
    bracket = plate.union(top_ear).union(lower_hook)

    ports = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, 0.020), (0.0, -0.014)])
        .rect(0.012, 0.024)
        .extrude(0.020, both=True)
    )
    screw_hole = cq.Workplane("YZ").center(0.0055, 0.056).circle(0.0018).extrude(0.020, both=True)
    latch_relief = (
        cq.Workplane("YZ").center(-0.0025, -0.040).rect(0.004, 0.012).extrude(0.020, both=True)
    )

    return bracket.cut(ports).cut(screw_hole).cut(latch_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="graphics_card", assets=ASSETS)

    model.material("shroud_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("graphite", rgba=(0.23, 0.24, 0.27, 1.0))
    model.material("brushed_aluminum", rgba=(0.66, 0.68, 0.72, 1.0))
    model.material("fan_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("gold_contact", rgba=(0.86, 0.72, 0.25, 1.0))
    model.material("trim_metal", rgba=(0.47, 0.49, 0.53, 1.0))
    model.material("bracket_metal", rgba=(0.71, 0.73, 0.76, 1.0))

    shroud_mesh = mesh_from_cadquery(_make_shroud_shape(), "gpu_shroud.obj", assets=ASSETS)
    backplate_mesh = mesh_from_cadquery(_make_backplate_shape(), "gpu_backplate.obj", assets=ASSETS)
    heatsink_mesh = mesh_from_cadquery(_make_heatsink_shape(), "gpu_heatsink.obj", assets=ASSETS)
    fan_mesh = mesh_from_cadquery(_make_fan_rotor_shape(), "gpu_fan_rotor.obj", assets=ASSETS)
    bracket_mesh = mesh_from_cadquery(_make_bracket_shape(), "gpu_bracket.obj", assets=ASSETS)

    body = model.part("gpu_body")
    body.visual(shroud_mesh, material="shroud_black")
    body.visual(backplate_mesh, material="graphite")
    body.visual(heatsink_mesh, material="brushed_aluminum")
    body.visual(
        Box((0.072, 0.006, 0.010)),
        origin=Origin(xyz=(-0.022, -0.006, -0.058)),
        material="gold_contact",
    )
    body.visual(
        Box((0.172, 0.0025, 0.006)),
        origin=Origin(xyz=(0.010, 0.0245, 0.044)),
        material="trim_metal",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_LEN, BODY_THICK, BODY_HEIGHT)),
        mass=1.55,
    )

    bracket = model.part("bracket")
    bracket.visual(bracket_mesh, material="bracket_metal")
    bracket.inertial = Inertial.from_geometry(
        Box((0.008, 0.019, 0.120)),
        mass=0.08,
        origin=Origin(xyz=(-0.003, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_bracket",
        ArticulationType.FIXED,
        parent="gpu_body",
        child="bracket",
        origin=Origin(xyz=(BRACKET_INSERT_X, 0.0, 0.0)),
    )

    for name, fan_x in (
        ("fan_left", FAN_XS[0]),
        ("fan_center", FAN_XS[1]),
        ("fan_right", FAN_XS[2]),
    ):
        fan = model.part(name)
        fan.visual(fan_mesh, material="fan_black")
        fan.inertial = Inertial.from_geometry(
            Box((FAN_ROTOR_R * 2.0, FAN_DEPTH, FAN_ROTOR_R * 2.0)),
            mass=0.035,
        )
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent="gpu_body",
            child=name,
            origin=Origin(xyz=(fan_x, FAN_CENTER_Y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=120.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("bracket", "gpu_body", axes="xy", min_overlap=0.001)
    ctx.expect_aabb_overlap("fan_left", "gpu_body", axes="xy", min_overlap=0.004)
    ctx.expect_aabb_overlap("fan_center", "gpu_body", axes="xy", min_overlap=0.004)
    ctx.expect_aabb_overlap("fan_right", "gpu_body", axes="xy", min_overlap=0.004)
    ctx.expect_origin_distance("fan_center", "gpu_body", axes="xy", max_dist=0.02)
    ctx.expect_origin_distance("fan_left", "fan_center", axes="xy", max_dist=0.09)
    ctx.expect_origin_distance("fan_center", "fan_right", axes="xy", max_dist=0.09)

    for pose in (
        {"fan_left_spin": 0.0, "fan_center_spin": 0.0, "fan_right_spin": 0.0},
        {"fan_left_spin": 1.15, "fan_center_spin": 2.20, "fan_right_spin": 0.55},
        {"fan_left_spin": 2.40, "fan_center_spin": 0.95, "fan_right_spin": 1.70},
    ):
        with ctx.pose(pose):
            ctx.expect_aabb_overlap("fan_left", "gpu_body", axes="xy", min_overlap=0.004)
            ctx.expect_aabb_overlap("fan_center", "gpu_body", axes="xy", min_overlap=0.004)
            ctx.expect_aabb_overlap("fan_right", "gpu_body", axes="xy", min_overlap=0.004)
            ctx.expect_origin_distance("fan_left", "fan_center", axes="xy", max_dist=0.09)
            ctx.expect_origin_distance("fan_center", "fan_right", axes="xy", max_dist=0.09)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
