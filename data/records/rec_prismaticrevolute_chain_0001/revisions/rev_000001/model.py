from __future__ import annotations

from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
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
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
BASE_L = 0.42
BASE_W = 0.22
BASE_T = 0.02
RAIL_L = 0.30
RAIL_W = 0.032
RAIL_H = 0.035
RAIL_Y = 0.07
REAR_STOP_L = 0.04
REAR_STOP_W = 0.18
REAR_STOP_H = 0.065
REAR_STOP_X = -0.17
FRONT_STOP_L = 0.03
FRONT_STOP_W = 0.10
FRONT_STOP_H = 0.025
FRONT_STOP_X = 0.18

SHOE_L = 0.12
SHOE_W = 0.03
SHOE_H = 0.02
BRIDGE_L = 0.13
BRIDGE_W = 0.12
BRIDGE_H = 0.014
BRIDGE_Z = SHOE_H + (BRIDGE_H / 2.0)
PEDESTAL_L = 0.04
PEDESTAL_W = 0.07
PEDESTAL_H = 0.016
PEDESTAL_X = -0.038
PEDESTAL_Z = SHOE_H + BRIDGE_H + (PEDESTAL_H / 2.0)
EAR_L = 0.018
EAR_W = 0.014
EAR_H = 0.03
EAR_X = -0.04
EAR_OFFSET_Y = 0.025
EAR_Z = SHOE_H + BRIDGE_H + PEDESTAL_H + (EAR_H / 2.0)
PRISMATIC_LOWER = -0.08
PRISMATIC_UPPER = 0.095

FLAP_L = 0.10
FLAP_W = 0.09
FLAP_T = 0.005
FLAP_X = -0.062
FLAP_Z = -0.0105
BARREL_R = 0.008
BARREL_VIS_L = 0.05
BARREL_COL_L = 0.03
TAB_L = 0.018
TAB_W = 0.016
TAB_T = 0.004
TAB_X = -0.012
TAB_Z = -0.0085
HANDLE_L = 0.024
HANDLE_W = 0.014
HANDLE_H = 0.008
HANDLE_X = -0.09
HANDLE_Z = -0.002
FLAP_UPPER = 1.3


def _require_cadquery():
    import cadquery as cq

    return cq


def _build_base_shape(cq):
    base_plate = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T).edges("|Z").fillet(0.004)
    left_rail = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_H)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, RAIL_Y, (BASE_T / 2.0) + (RAIL_H / 2.0)))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_H)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, -RAIL_Y, (BASE_T / 2.0) + (RAIL_H / 2.0)))
    )
    rear_stop = (
        cq.Workplane("XY")
        .box(REAR_STOP_L, REAR_STOP_W, REAR_STOP_H)
        .edges("|Z")
        .fillet(0.004)
        .translate((REAR_STOP_X, 0.0, (BASE_T / 2.0) + (REAR_STOP_H / 2.0)))
    )
    front_stop = (
        cq.Workplane("XY")
        .box(FRONT_STOP_L, FRONT_STOP_W, FRONT_STOP_H)
        .edges("|Z")
        .fillet(0.003)
        .translate((FRONT_STOP_X, 0.0, (BASE_T / 2.0) + (FRONT_STOP_H / 2.0)))
    )
    datum_pad = (
        cq.Workplane("XY")
        .box(0.05, 0.06, 0.012)
        .edges("|Z")
        .fillet(0.002)
        .translate((-0.12, 0.0, (BASE_T / 2.0) + 0.006))
    )
    return (
        base_plate.union(left_rail)
        .union(right_rail)
        .union(rear_stop)
        .union(front_stop)
        .union(datum_pad)
    )


def _build_carriage_shape(cq):
    left_shoe = (
        cq.Workplane("XY")
        .box(SHOE_L, SHOE_W, SHOE_H)
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.0, 0.0, SHOE_H / 2.0))
    )
    right_shoe = (
        cq.Workplane("XY")
        .box(SHOE_L, SHOE_W, SHOE_H)
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.0, -2.0 * RAIL_Y, SHOE_H / 2.0))
    )
    bridge = (
        cq.Workplane("XY")
        .box(BRIDGE_L, BRIDGE_W, BRIDGE_H)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, -RAIL_Y, BRIDGE_Z))
    )
    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_L, PEDESTAL_W, PEDESTAL_H)
        .edges("|Z")
        .fillet(0.002)
        .translate((PEDESTAL_X, -RAIL_Y, PEDESTAL_Z))
    )
    upper_web = (
        cq.Workplane("XY")
        .box(0.02, 0.05, 0.01)
        .edges("|Z")
        .fillet(0.0015)
        .translate((-0.012, -RAIL_Y, 0.055))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(EAR_L, EAR_W, EAR_H)
        .edges("|Z")
        .fillet(0.0015)
        .translate((EAR_X, -RAIL_Y + EAR_OFFSET_Y, EAR_Z))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(EAR_L, EAR_W, EAR_H)
        .edges("|Z")
        .fillet(0.0015)
        .translate((EAR_X, -RAIL_Y - EAR_OFFSET_Y, EAR_Z))
    )
    return (
        left_shoe.union(right_shoe)
        .union(bridge)
        .union(pedestal)
        .union(upper_web)
        .union(left_ear)
        .union(right_ear)
    )


def _build_flap_shape(cq):
    panel = (
        cq.Workplane("XY")
        .box(FLAP_L, FLAP_W, FLAP_T)
        .edges("|Z")
        .fillet(0.005)
        .translate((FLAP_X, 0.0, FLAP_Z))
    )
    hinge_barrel = (
        cq.Workplane("XY").cylinder(BARREL_VIS_L, BARREL_R).rotate((0, 0, 0), (1, 0, 0), 90)
    )
    connector = (
        cq.Workplane("XY")
        .box(TAB_L, TAB_W, TAB_T)
        .edges("|Z")
        .fillet(0.001)
        .translate((TAB_X, 0.0, TAB_Z))
    )
    handle = (
        cq.Workplane("XY")
        .box(HANDLE_L, HANDLE_W, HANDLE_H)
        .edges("|Z")
        .fillet(0.0015)
        .translate((HANDLE_X, 0.0, HANDLE_Z))
    )
    return hinge_barrel.union(connector).union(panel).union(handle)


def build_object_model() -> ArticulatedObject:
    cq = _require_cadquery()

    model = ArticulatedObject(name="inspection_fixture", assets=ASSETS)
    model.material("fixture_base", rgba=(0.42, 0.44, 0.48, 1.0))
    model.material("fixture_carriage", rgba=(0.78, 0.46, 0.14, 1.0))
    model.material("fixture_flap", rgba=(0.92, 0.77, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(cq), MESH_DIR / "fixture_base.obj"),
        material="fixture_base",
    )






    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.08)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(cq), MESH_DIR / "carriage.obj"),
        material="fixture_carriage",
    )







    carriage.inertial = Inertial.from_geometry(
        Box((0.14, 0.17, 0.08)),
        mass=1.8,
        origin=Origin(xyz=(-0.01, -RAIL_Y, 0.04)),
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_build_flap_shape(cq), MESH_DIR / "inspection_flap.obj"),
        material="fixture_flap",
    )



    flap.inertial = Inertial.from_geometry(
        Box((0.12, 0.09, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(-0.055, 0.0, -0.006)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent="base",
        child="carriage",
        origin=Origin(xyz=(0.0, RAIL_Y, (BASE_T / 2.0) + RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=PRISMATIC_LOWER,
            upper=PRISMATIC_UPPER,
            effort=120.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "carriage_to_flap",
        ArticulationType.REVOLUTE,
        parent="carriage",
        child="flap",
        origin=Origin(xyz=(EAR_X, -RAIL_Y, EAR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=FLAP_UPPER, effort=10.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.09)
    ctx.expect_joint_motion_axis(
        "base_to_carriage", "carriage", world_axis="x", direction="positive", min_delta=0.03
    )

    ctx.expect_aabb_overlap("flap", "carriage", axes="xy", min_overlap=0.03)
    ctx.expect_origin_distance("flap", "carriage", axes="xy", max_dist=0.085)
    ctx.expect_joint_motion_axis(
        "carriage_to_flap", "flap", world_axis="z", direction="positive", min_delta=0.03
    )

    with ctx.pose(base_to_carriage=PRISMATIC_LOWER):
        ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_overlap("flap", "carriage", axes="xy", min_overlap=0.03)

    with ctx.pose(base_to_carriage=PRISMATIC_UPPER):
        ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_overlap("flap", "carriage", axes="xy", min_overlap=0.03)

    with ctx.pose(carriage_to_flap=FLAP_UPPER):
        ctx.expect_origin_gap("flap", "carriage", axis="z", min_gap=0.03)

    with ctx.pose(base_to_carriage=PRISMATIC_UPPER, carriage_to_flap=FLAP_UPPER):
        ctx.expect_origin_gap("flap", "base", axis="z", min_gap=0.04)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
