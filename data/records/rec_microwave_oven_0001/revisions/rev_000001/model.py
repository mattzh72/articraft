from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    VentGrilleGeometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _grille_panel_geometry(
    panel_size,
    thickness,
    *,
    frame,
    slat_pitch,
    slat_width,
    slat_angle_deg,
    corner_radius,
    center=True,
):
    geom = VentGrilleGeometry(
        panel_size,
        frame=frame,
        face_thickness=thickness,
        duct_depth=max(0.0015, thickness * 0.75),
        duct_wall=max(0.001, min(frame * 0.45, thickness * 0.75)),
        slat_pitch=slat_pitch,
        slat_width=slat_width,
        slat_angle_deg=slat_angle_deg,
        corner_radius=corner_radius,
    )
    if not center:
        geom.translate(0.0, 0.0, thickness * 0.5)
    return geom


def _materials() -> dict[str, Material]:
    return {
        "stainless": Material(name="stainless", rgba=(0.74, 0.75, 0.77, 1.0)),
        "black_glass": Material(name="black_glass", rgba=(0.08, 0.09, 0.10, 1.0)),
        "smoked_glass": Material(name="smoked_glass", rgba=(0.22, 0.25, 0.28, 0.32)),
        "charcoal": Material(name="charcoal", rgba=(0.18, 0.19, 0.21, 1.0)),
        "graphite": Material(name="graphite", rgba=(0.28, 0.29, 0.31, 1.0)),
        "aluminum": Material(name="aluminum", rgba=(0.84, 0.85, 0.86, 1.0)),
        "rubber": Material(name="rubber", rgba=(0.06, 0.06, 0.06, 1.0)),
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="microwave_oven", assets=ASSETS)
    mats = _materials()
    model.materials.extend(mats.values())

    body = model.part("body_shell")
    body.visual(
        Box((0.49, 0.39, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=mats["stainless"],
        name="bottom_panel",
    )
    body.visual(
        Box((0.49, 0.39, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.284)),
        material=mats["stainless"],
        name="top_panel",
    )
    body.visual(
        Box((0.012, 0.39, 0.264)),
        origin=Origin(xyz=(-0.239, 0.0, 0.152)),
        material=mats["stainless"],
        name="left_shell",
    )
    body.visual(
        Box((0.012, 0.39, 0.264)),
        origin=Origin(xyz=(0.239, 0.0, 0.152)),
        material=mats["stainless"],
        name="right_shell",
    )
    body.visual(
        Box((0.466, 0.012, 0.264)),
        origin=Origin(xyz=(0.0, -0.189, 0.152)),
        material=mats["stainless"],
        name="rear_shell",
    )
    body.visual(
        Box((0.378, 0.008, 0.010)),
        origin=Origin(xyz=(-0.044, 0.191, 0.016)),
        material=mats["charcoal"],
        name="front_bezel_bottom",
    )
    body.visual(
        Box((0.378, 0.008, 0.010)),
        origin=Origin(xyz=(-0.044, 0.191, 0.274)),
        material=mats["charcoal"],
        name="front_bezel_top",
    )
    body.visual(
        Box((0.010, 0.008, 0.248)),
        origin=Origin(xyz=(0.139, 0.191, 0.145)),
        material=mats["charcoal"],
        name="front_bezel_right",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Box((0.030, 0.018, 0.008)),
                origin=Origin(xyz=(x_sign * 0.195, y_sign * 0.145, 0.004)),
                material=mats["rubber"],
                name=f"foot_{'r' if x_sign > 0 else 'l'}_{'f' if y_sign > 0 else 'b'}",
            )

    rear_vent = mesh_from_geometry(
        _grille_panel_geometry(
            panel_size=(0.24, 0.10),
            thickness=0.003,
            frame=0.008,
            slat_pitch=0.018,
            slat_width=0.008,
            slat_angle_deg=28.0,
            corner_radius=0.004,
            center=True,
        ),
        ASSETS.mesh_path("microwave_rear_vent.obj"),
    )
    body.visual(
        rear_vent,
        origin=Origin(xyz=(0.01, -0.1965, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["charcoal"],
        name="rear_vent",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.49, 0.39, 0.29)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )

    cavity = model.part("cavity_liner")
    cavity.visual(
        Box((0.358, 0.325, 0.006)),
        origin=Origin(xyz=(0.182, 0.0, -0.112)),
        material=mats["charcoal"],
        name="floor",
    )
    cavity.visual(
        Box((0.358, 0.325, 0.006)),
        origin=Origin(xyz=(0.182, 0.0, 0.112)),
        material=mats["charcoal"],
        name="ceiling",
    )
    cavity.visual(
        Box((0.006, 0.325, 0.228)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=mats["charcoal"],
        name="left_wall",
    )
    cavity.visual(
        Box((0.006, 0.325, 0.228)),
        origin=Origin(xyz=(0.361, 0.0, 0.0)),
        material=mats["charcoal"],
        name="right_wall",
    )
    cavity.visual(
        Box((0.358, 0.006, 0.228)),
        origin=Origin(xyz=(0.182, -0.159, 0.0)),
        material=mats["charcoal"],
        name="back_wall",
    )
    cavity.visual(
        Box((0.040, 0.004, 0.032)),
        origin=Origin(xyz=(0.343, -0.050, 0.070)),
        material=mats["black_glass"],
        name="waveguide_cover",
    )
    cavity.inertial = Inertial.from_geometry(
        Box((0.358, 0.325, 0.25)),
        mass=1.0,
        origin=Origin(xyz=(0.182, 0.0, -0.005)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.134, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=mats["smoked_glass"],
        name="glass_plate",
    )
    turntable.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=mats["aluminum"],
        name="hub",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.134, length=0.004),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.084, 0.008, 0.258)),
        origin=Origin(xyz=(-0.042, 0.0, 0.0)),
        material=mats["charcoal"],
        name="panel_face",
    )
    control_panel.visual(
        Box((0.068, 0.002, 0.036)),
        origin=Origin(xyz=(-0.042, 0.005, 0.085)),
        material=mats["aluminum"],
        name="display_trim",
    )
    control_panel.visual(
        Box((0.056, 0.004, 0.024)),
        origin=Origin(xyz=(-0.042, 0.006, 0.085)),
        material=mats["black_glass"],
        name="display_lens",
    )
    for i, x in enumerate((-0.064, -0.042, -0.020), start=1):
        control_panel.visual(
            Box((0.018, 0.004, 0.012)),
            origin=Origin(xyz=(x, 0.006, 0.030)),
            material=mats["aluminum"],
            name=f"button_{i}",
        )
    control_panel.visual(
        Box((0.060, 0.002, 0.018)),
        origin=Origin(xyz=(-0.042, 0.005, -0.074)),
        material=mats["aluminum"],
        name="brand_badge",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((0.084, 0.030, 0.258)),
        mass=0.7,
        origin=Origin(xyz=(-0.042, 0.010, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((0.050, 0.032, 0.242)),
        origin=Origin(xyz=(0.027, 0.017, 0.0)),
        material=mats["black_glass"],
        name="frame_left",
    )
    door.visual(
        Box((0.050, 0.032, 0.242)),
        origin=Origin(xyz=(0.337, 0.017, 0.0)),
        material=mats["black_glass"],
        name="frame_right",
    )
    door.visual(
        Box((0.290, 0.032, 0.042)),
        origin=Origin(xyz=(0.182, 0.017, 0.100)),
        material=mats["black_glass"],
        name="frame_top",
    )
    door.visual(
        Box((0.290, 0.032, 0.042)),
        origin=Origin(xyz=(0.182, 0.017, -0.100)),
        material=mats["black_glass"],
        name="frame_bottom",
    )
    door.visual(
        Box((0.252, 0.004, 0.158)),
        origin=Origin(xyz=(0.182, 0.010, 0.0)),
        material=mats["smoked_glass"],
        name="window",
    )
    door.visual(
        Box((0.024, 0.010, 0.022)),
        origin=Origin(xyz=(0.315, 0.035, 0.078)),
        material=mats["aluminum"],
        name="handle_mount_top",
    )
    door.visual(
        Box((0.024, 0.010, 0.022)),
        origin=Origin(xyz=(0.315, 0.035, -0.078)),
        material=mats["aluminum"],
        name="handle_mount_bottom",
    )
    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.315, 0.035, 0.078),
                (0.328, 0.050, 0.078),
                (0.340, 0.060, 0.040),
                (0.340, 0.060, -0.040),
                (0.328, 0.050, -0.078),
                (0.315, 0.035, -0.078),
            ],
            radius=0.0055,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        ASSETS.mesh_path("microwave_door_handle.obj"),
    )
    door.visual(handle_mesh, material=mats["aluminum"], name="handle")
    door.inertial = Inertial.from_geometry(
        Box((0.362, 0.032, 0.242)),
        mass=2.1,
        origin=Origin(xyz=(0.182, 0.017, 0.0)),
    )

    upper_knob = model.part("upper_knob")
    upper_knob.visual(
        Cylinder(radius=0.019, length=0.025),
        origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["graphite"],
        name="knob_body",
    )
    upper_knob.visual(
        Cylinder(radius=0.014, length=0.005),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["aluminum"],
        name="knob_cap",
    )
    upper_knob.visual(
        Box((0.003, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, 0.0325, 0.013)),
        material=mats["aluminum"],
        name="indicator",
    )
    upper_knob.inertial = Inertial.from_geometry(
        Box((0.038, 0.030, 0.038)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
    )

    lower_knob = model.part("lower_knob")
    lower_knob.visual(
        Cylinder(radius=0.021, length=0.025),
        origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["graphite"],
        name="knob_body",
    )
    lower_knob.visual(
        Cylinder(radius=0.015, length=0.005),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["aluminum"],
        name="knob_cap",
    )
    lower_knob.visual(
        Box((0.003, 0.002, 0.013)),
        origin=Origin(xyz=(0.0, 0.0325, 0.014)),
        material=mats["aluminum"],
        name="indicator",
    )
    lower_knob.inertial = Inertial.from_geometry(
        Box((0.042, 0.030, 0.042)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
    )

    model.articulation(
        "cavity_mount",
        ArticulationType.FIXED,
        parent="body_shell",
        child="cavity_liner",
        origin=Origin(xyz=(-0.229, 0.0, 0.145)),
    )
    model.articulation(
        "turntable_mount",
        ArticulationType.FIXED,
        parent="cavity_liner",
        child="turntable",
        origin=Origin(xyz=(0.182, 0.0, -0.108)),
    )
    model.articulation(
        "panel_mount",
        ArticulationType.FIXED,
        parent="body_shell",
        child="control_panel",
        origin=Origin(xyz=(0.233, 0.191, 0.145)),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent="body_shell",
        child="door",
        origin=Origin(xyz=(-0.240, 0.198, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.60,
        ),
    )
    model.articulation(
        "upper_knob_joint",
        ArticulationType.REVOLUTE,
        parent="control_panel",
        child="upper_knob",
        origin=Origin(xyz=(-0.042, 0.004, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=5.0,
            lower=-0.70,
            upper=2.60,
        ),
    )
    model.articulation(
        "lower_knob_joint",
        ArticulationType.REVOLUTE,
        parent="control_panel",
        child="lower_knob",
        origin=Origin(xyz=(-0.042, 0.004, -0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=5.0,
            lower=0.0,
            upper=5.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("control_panel", "body_shell", axes="xy", min_overlap=0.005)
    ctx.expect_aabb_overlap("cavity_liner", "body_shell", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_overlap("turntable", "cavity_liner", axes="xy", min_overlap=0.08)
    ctx.expect_origin_distance("turntable", "body_shell", axes="xy", max_dist=0.06)
    ctx.expect_origin_distance("door", "body_shell", axes="xy", max_dist=0.33)
    ctx.expect_aabb_gap("upper_knob", "lower_knob", axis="z", max_gap=0.08, max_penetration=0.0)
    ctx.expect_origin_distance("upper_knob", "control_panel", axes="xy", max_dist=0.05)
    ctx.expect_origin_distance("lower_knob", "control_panel", axes="xy", max_dist=0.05)
    ctx.expect_joint_motion_axis(
        "door_hinge",
        "door",
        world_axis="y",
        direction="positive",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "door_hinge",
        "door",
        world_axis="x",
        direction="negative",
        min_delta=0.03,
    )

    with ctx.pose(door_hinge=0.0):
        ctx.expect_origin_distance("door", "body_shell", axes="xy", max_dist=0.33)
        ctx.expect_origin_distance("turntable", "body_shell", axes="xy", max_dist=0.06)

    with ctx.pose(door_hinge=1.60):
        ctx.expect_origin_distance("door", "body_shell", axes="xy", max_dist=0.46)
        ctx.expect_origin_distance("turntable", "body_shell", axes="xy", max_dist=0.06)

    with ctx.pose(upper_knob_joint=-0.70, lower_knob_joint=0.0):
        ctx.expect_origin_distance("upper_knob", "control_panel", axes="xy", max_dist=0.05)
        ctx.expect_origin_distance("lower_knob", "control_panel", axes="xy", max_dist=0.05)

    with ctx.pose(upper_knob_joint=2.60, lower_knob_joint=5.50):
        ctx.expect_origin_distance("upper_knob", "control_panel", axes="xy", max_dist=0.05)
        ctx.expect_origin_distance("lower_knob", "control_panel", axes="xy", max_dist=0.05)
        ctx.expect_aabb_gap("upper_knob", "lower_knob", axis="z", max_gap=0.08, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
