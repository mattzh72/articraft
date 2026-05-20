from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
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
def _polar_xy(radius: float, angle_deg: float) -> tuple[float, float]:
    angle_rad = math.radians(angle_deg)
    return (radius * math.cos(angle_rad), radius * math.sin(angle_rad))


def _rotated_box(
    size_xyz: tuple[float, float, float],
    center_xyz: tuple[float, float, float],
    rz_deg: float,
):
    return (
        cq.Workplane("XY")
        .box(*size_xyz)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), rz_deg)
        .translate(center_xyz)
    )


def _make_support_frame_shape():
    base_t = 0.012
    plate = cq.Workplane("XY").box(0.22, 0.16, base_t).translate((0.0, 0.0, base_t / 2.0))
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.078, -0.052),
                (-0.078, 0.052),
                (0.078, -0.052),
                (0.078, 0.052),
            ]
        )
        .cboreHole(0.008, 0.015, 0.003)
    )

    frame = plate

    for y_sign in (-1.0, 1.0):
        frame = frame.union(
            cq.Workplane("XY").box(0.112, 0.014, 0.018).translate((0.0, y_sign * 0.046, 0.021))
        )

    for x_sign in (-1.0, 1.0):
        frame = frame.union(
            cq.Workplane("XY").box(0.018, 0.070, 0.016).translate((x_sign * 0.052, 0.0, 0.020))
        )

    spindle = cq.Workplane("XY").circle(0.020).extrude(0.022).translate((0.0, 0.0, base_t))
    lower_flange = (
        cq.Workplane("XY").circle(0.038).circle(0.024).extrude(0.006).translate((0.0, 0.0, base_t))
    )
    frame = frame.union(spindle).union(lower_flange)

    for angle_deg in (-75.0, 75.0):
        stop_x, stop_y = _polar_xy(0.064, angle_deg)
        frame = frame.union(_rotated_box((0.014, 0.010, 0.018), (stop_x, stop_y, 0.021), angle_deg))

    return frame


def _make_turntable_shape():
    disk = cq.Workplane("XY").circle(0.075).extrude(0.014)
    disk = disk.faces(">Z").workplane(centerOption="CenterOfMass").hole(0.044)
    disk = (
        disk.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([_polar_xy(0.048, angle_deg) for angle_deg in range(0, 360, 60)])
        .cboreHole(0.0055, 0.0105, 0.003)
    )

    hub_ring = (
        cq.Workplane("XY").circle(0.032).circle(0.022).extrude(0.015).translate((0.0, 0.0, -0.015))
    )
    underside_rib = cq.Workplane("XY").box(0.030, 0.010, 0.006).translate((0.040, 0.0, -0.004))
    stop_lug = cq.Workplane("XY").box(0.026, 0.016, 0.010).translate((0.082, 0.0, -0.006))
    index_notch = cq.Workplane("XY").box(0.012, 0.005, 0.008).translate((0.083, 0.0, 0.010))

    return disk.union(hub_ring).union(underside_rib).union(stop_lug).cut(index_notch)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rotary_indexing_base", assets=ASSETS)

    model.material("frame_charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("turntable_steel", rgba=(0.64, 0.66, 0.68, 1.0))
    model.material("pointer_red", rgba=(0.72, 0.13, 0.10, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(_make_support_frame_shape(), "support_frame.obj", assets=ASSETS),
        material="frame_charcoal",
    )






    support_frame.inertial = Inertial.from_geometry(
        Box((0.22, 0.16, 0.05)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        mesh_from_cadquery(_make_turntable_shape(), "turntable.obj", assets=ASSETS),
        material="turntable_steel",
    )


    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.071, length=0.020),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    index_pointer = model.part("index_pointer")
    index_pointer.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").box(0.028, 0.008, 0.006).edges("|Z").fillet(0.0015),
            "index_pointer.obj",
            assets=ASSETS,
        ),
        material="pointer_red",
    )

    index_pointer.inertial = Inertial.from_geometry(Box((0.028, 0.008, 0.006)), mass=0.05)

    model.articulation(
        "support_to_turntable",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.10, effort=20.0, velocity=2.5),
    )
    model.articulation(
        "turntable_to_pointer",
        ArticulationType.FIXED,
        parent=turntable,
        child=index_pointer,
        origin=Origin(xyz=(0.058, 0.0, 0.017)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)
    ctx.expect_joint_motion_axis(
        "support_to_turntable",
        "index_pointer",
        world_axis="y",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_origin_distance("turntable", "support_frame", axes="xy", max_dist=0.001)
    ctx.expect_aabb_overlap("turntable", "support_frame", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_gap("turntable", "support_frame", axis="z", max_gap=0.006, max_penetration=0.0)
    ctx.expect_origin_gap("index_pointer", "turntable", axis="z", min_gap=0.0)
    ctx.expect_aabb_overlap("index_pointer", "turntable", axes="xy", min_overlap=0.004)
    ctx.expect_aabb_gap("index_pointer", "turntable", axis="z", max_gap=0.001, max_penetration=0.0)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
