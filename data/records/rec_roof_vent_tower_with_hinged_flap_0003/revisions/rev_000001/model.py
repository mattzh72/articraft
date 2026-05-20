from __future__ import annotations

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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_roof_vent_tower", assets=ASSETS)

    painted_shell = model.material("painted_shell", rgba=(0.71, 0.73, 0.75, 1.0))
    satin_trim = model.material("satin_trim", rgba=(0.60, 0.63, 0.66, 1.0))
    dark_flashing = model.material("dark_flashing", rgba=(0.29, 0.31, 0.33, 1.0))
    shadow_cavity = model.material("shadow_cavity", rgba=(0.11, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    hardware = model.material("hardware", rgba=(0.56, 0.58, 0.60, 1.0))

    body = model.part("tower_body")

    body.visual(
        Box((0.74, 0.52, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_flashing,
        name="roof_flashing",
    )
    body.visual(
        Box((0.44, 0.32, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_flashing,
        name="mount_curb",
    )

    shell_center_z = 0.26
    shell_height = 0.46
    body.visual(
        Box((0.018, 0.40, shell_height)),
        origin=Origin(xyz=(-0.151, 0.0, shell_center_z)),
        material=painted_shell,
        name="back_panel",
    )
    body.visual(
        Box((0.304, 0.018, shell_height)),
        origin=Origin(xyz=(0.0, -0.191, shell_center_z)),
        material=painted_shell,
        name="left_side_panel",
    )
    body.visual(
        Box((0.304, 0.018, shell_height)),
        origin=Origin(xyz=(0.0, 0.191, shell_center_z)),
        material=painted_shell,
        name="right_side_panel",
    )
    body.visual(
        Box((0.018, 0.40, 0.116)),
        origin=Origin(xyz=(0.151, 0.0, 0.088)),
        material=painted_shell,
        name="front_plinth",
    )
    body.visual(
        Box((0.018, 0.040, 0.244)),
        origin=Origin(xyz=(0.151, -0.180, 0.274)),
        material=painted_shell,
        name="left_front_jamb",
    )
    body.visual(
        Box((0.018, 0.040, 0.244)),
        origin=Origin(xyz=(0.151, 0.180, 0.274)),
        material=painted_shell,
        name="right_front_jamb",
    )
    body.visual(
        Box((0.018, 0.40, 0.092)),
        origin=Origin(xyz=(0.151, 0.0, 0.444)),
        material=painted_shell,
        name="front_head_panel",
    )
    body.visual(
        Box((0.34, 0.42, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.506)),
        material=painted_shell,
        name="top_cap",
    )
    body.visual(
        Box((0.332, 0.392, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.489)),
        material=satin_trim,
        name="seam_band",
    )
    body.visual(
        Box((0.010, 0.010, 0.410)),
        origin=Origin(xyz=(0.158, -0.186, 0.269)),
        material=satin_trim,
        name="left_corner_trim",
    )
    body.visual(
        Box((0.010, 0.010, 0.410)),
        origin=Origin(xyz=(0.158, 0.186, 0.269)),
        material=satin_trim,
        name="right_corner_trim",
    )

    body.visual(
        Box((0.012, 0.320, 0.026)),
        origin=Origin(xyz=(0.165, 0.0, 0.398)),
        material=satin_trim,
        name="opening_frame_top",
    )
    body.visual(
        Box((0.012, 0.320, 0.026)),
        origin=Origin(xyz=(0.165, 0.0, 0.152)),
        material=satin_trim,
        name="opening_sill",
    )
    body.visual(
        Box((0.012, 0.026, 0.220)),
        origin=Origin(xyz=(0.165, -0.147, 0.275)),
        material=satin_trim,
        name="opening_frame_left",
    )
    body.visual(
        Box((0.012, 0.026, 0.220)),
        origin=Origin(xyz=(0.165, 0.147, 0.275)),
        material=satin_trim,
        name="opening_frame_right",
    )
    body.visual(
        Box((0.018, 0.286, 0.188)),
        origin=Origin(xyz=(0.150, 0.0, 0.275)),
        material=rubber,
        name="frame_break",
    )
    body.visual(
        Box((0.10, 0.262, 0.184)),
        origin=Origin(xyz=(0.108, 0.0, 0.275)),
        material=shadow_cavity,
        name="outlet_throat",
    )
    body.visual(
        Box((0.090, 0.238, 0.010)),
        origin=Origin(xyz=(0.108, 0.0, 0.344), rpy=(0.0, -0.28, 0.0)),
        material=shadow_cavity,
        name="upper_baffle",
    )
    body.visual(
        Box((0.070, 0.226, 0.010)),
        origin=Origin(xyz=(0.100, 0.0, 0.205), rpy=(0.0, 0.22, 0.0)),
        material=shadow_cavity,
        name="lower_baffle",
    )

    hinge_axis_x = 0.176
    hinge_axis_z = 0.430
    barrel_radius = 0.007
    body.visual(
        Box((0.010, 0.180, 0.018)),
        origin=Origin(xyz=(0.160, 0.0, hinge_axis_z)),
        material=satin_trim,
        name="hinge_header",
    )
    body.visual(
        Box((0.018, 0.056, 0.018)),
        origin=Origin(xyz=(0.166, -0.118, hinge_axis_z)),
        material=satin_trim,
        name="left_hinge_mount",
    )
    body.visual(
        Box((0.018, 0.056, 0.018)),
        origin=Origin(xyz=(0.166, 0.118, hinge_axis_z)),
        material=satin_trim,
        name="right_hinge_mount",
    )
    body.visual(
        Cylinder(radius=barrel_radius, length=0.056),
        origin=Origin(
            xyz=(hinge_axis_x, -0.118, hinge_axis_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="body_hinge_left",
    )
    body.visual(
        Cylinder(radius=barrel_radius, length=0.056),
        origin=Origin(
            xyz=(hinge_axis_x, 0.118, hinge_axis_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="body_hinge_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.74, 0.52, 0.55)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
    )

    flap = model.part("weather_flap")

    flap_pitch = 0.55
    hood_length = 0.22
    flap.visual(
        Cylinder(radius=0.007, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="flap_hinge_barrel",
    )
    flap.visual(
        Box((0.028, 0.118, 0.006)),
        origin=Origin(xyz=(0.020, 0.0, -0.004)),
        material=satin_trim,
        name="hinge_leaf",
    )
    flap.visual(
        Box((0.060, 0.260, 0.008)),
        origin=Origin(xyz=(0.036, 0.0, -0.008)),
        material=satin_trim,
        name="rear_bridge",
    )
    flap.visual(
        Box((hood_length, 0.356, 0.012)),
        origin=Origin(xyz=(0.110, 0.0, -0.064), rpy=(0.0, flap_pitch, 0.0)),
        material=satin_trim,
        name="hood_skin",
    )
    flap.visual(
        Box((0.176, 0.248, 0.010)),
        origin=Origin(xyz=(0.112, 0.0, -0.082), rpy=(0.0, flap_pitch, 0.0)),
        material=shadow_cavity,
        name="hood_underside",
    )
    flap.visual(
        Box((0.024, 0.336, 0.026)),
        origin=Origin(xyz=(0.204, 0.0, -0.128), rpy=(0.0, flap_pitch, 0.0)),
        material=painted_shell,
        name="weather_nose",
    )
    flap.visual(
        Box((0.184, 0.012, 0.072)),
        origin=Origin(xyz=(0.116, -0.170, -0.086), rpy=(0.0, flap_pitch, 0.0)),
        material=painted_shell,
        name="left_cheek",
    )
    flap.visual(
        Box((0.184, 0.012, 0.072)),
        origin=Origin(xyz=(0.116, 0.170, -0.086), rpy=(0.0, flap_pitch, 0.0)),
        material=painted_shell,
        name="right_cheek",
    )
    flap.visual(
        Box((0.006, 0.286, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, -0.026)),
        material=rubber,
        name="rear_seal_strip",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.24, 0.36, 0.12)),
        mass=3.2,
        origin=Origin(xyz=(0.116, 0.0, -0.076)),
    )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent="tower_body",
        child="weather_flap",
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.5,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Add narrow allowances here when conservative QC reports acceptable cases.
    # Add prompt-specific expect_* semantic checks below; they are the main regressions.
    ctx.expect_origin_distance("weather_flap", "tower_body", axes="y", max_dist=0.001)
    ctx.expect_aabb_overlap("weather_flap", "tower_body", axes="y", min_overlap=0.30)
    ctx.expect_aabb_gap(
        "tower_body",
        "weather_flap",
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="body_hinge_right",
        negative_elem="flap_hinge_barrel",
        name="right_hinge_barrels_meet_cleanly",
    )
    ctx.expect_aabb_gap(
        "weather_flap",
        "tower_body",
        axis="x",
        max_gap=0.020,
        max_penetration=0.0,
        positive_elem="rear_seal_strip",
        negative_elem="opening_frame_top",
        name="closed_flap_seal_sits_just_off_frame",
    )
    ctx.expect_aabb_gap(
        "weather_flap",
        "tower_body",
        axis="x",
        max_gap=0.010,
        max_penetration=0.0,
        positive_elem="flap_hinge_barrel",
        negative_elem="hinge_header",
        name="pivot_hardware_sits_close_to_header",
    )
    ctx.expect_joint_motion_axis(
        "flap_hinge",
        "weather_flap",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )
    with ctx.pose(flap_hinge=0.95):
        ctx.expect_aabb_gap(
            "weather_flap",
            "tower_body",
            axis="x",
            min_gap=0.025,
            positive_elem="weather_nose",
            negative_elem="opening_frame_top",
            name="opened_flap_clears_frame",
        )
        ctx.expect_aabb_gap(
            "weather_flap",
            "tower_body",
            axis="z",
            min_gap=0.040,
            positive_elem="weather_nose",
            negative_elem="opening_frame_top",
            name="opened_flap_lifts_clear_of_frame",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
