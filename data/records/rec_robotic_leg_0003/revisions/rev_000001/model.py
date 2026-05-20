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
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_leg", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_titanium = model.material("satin_titanium", rgba=(0.63, 0.65, 0.69, 1.0))
    satin_charcoal = model.material("satin_charcoal", rgba=(0.28, 0.30, 0.33, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    hip_gap = 0.0740
    hip_barrel_length = 0.0740
    hip_barrel_radius = 0.0185
    hip_cheek_thickness = 0.012

    knee_gap = 0.0560
    knee_barrel_length = 0.0564
    knee_barrel_radius = 0.0165
    knee_cheek_thickness = 0.010

    ankle_gap = 0.0780
    ankle_barrel_length = 0.0784
    ankle_barrel_radius = 0.0145
    ankle_cheek_thickness = 0.010

    def _section_xy(
        z: float,
        width_x: float,
        depth_y: float,
        *,
        x_shift: float = 0.0,
        y_shift: float = 0.0,
        exponent: float = 3.0,
        segments: int = 40,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_shift + px, y_shift + py, z)
            for px, py in superellipse_profile(
                width_x,
                depth_y,
                exponent=exponent,
                segments=segments,
            )
        ]

    def _section_yz(
        x: float,
        width_y: float,
        height_z: float,
        *,
        y_shift: float = 0.0,
        z_shift: float = 0.0,
        exponent: float = 3.0,
        segments: int = 36,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y_shift + py, z_shift + pz)
            for py, pz in superellipse_profile(
                width_y,
                height_z,
                exponent=exponent,
                segments=segments,
            )
        ]

    def _loft_mesh(name: str, sections: list[list[tuple[float, float, float]]]):
        geom = repair_loft(
            SectionLoftSpec(
                sections=tuple(sections),
                cap=True,
                solid=True,
            ),
            repair="auto",
        )
        return mesh_from_geometry(geom, ASSETS.mesh_path(name))

    hip_shell = _loft_mesh(
        "hip_shell.obj",
        [
            _section_xy(0.024, 0.086, 0.070, x_shift=0.004, exponent=3.2),
            _section_xy(0.046, 0.102, 0.082, x_shift=0.006, exponent=3.1),
            _section_xy(0.070, 0.112, 0.086, x_shift=0.006, exponent=3.0),
            _section_xy(0.094, 0.098, 0.080, x_shift=0.004, exponent=3.2),
        ],
    )
    thigh_shell = _loft_mesh(
        "thigh_shell.obj",
        [
            _section_xy(-0.014, 0.056, 0.048, x_shift=0.002, exponent=3.0),
            _section_xy(-0.055, 0.070, 0.058, x_shift=0.005, exponent=3.0),
            _section_xy(-0.115, 0.078, 0.061, x_shift=0.006, exponent=2.9),
            _section_xy(-0.162, 0.070, 0.056, x_shift=0.004, exponent=3.0),
            _section_xy(-0.186, 0.058, 0.049, x_shift=0.002, exponent=3.2),
        ],
    )
    shin_shell = _loft_mesh(
        "shin_shell.obj",
        [
            _section_xy(-0.014, 0.054, 0.045, x_shift=0.004, exponent=3.2),
            _section_xy(-0.060, 0.065, 0.052, x_shift=0.008, exponent=3.0),
            _section_xy(-0.118, 0.072, 0.055, x_shift=0.010, exponent=2.9),
            _section_xy(-0.164, 0.060, 0.048, x_shift=0.007, exponent=3.0),
            _section_xy(-0.186, 0.050, 0.041, x_shift=0.004, exponent=3.2),
        ],
    )
    foot_shell = _loft_mesh(
        "foot_shell.obj",
        [
            _section_yz(0.008, 0.052, 0.024, z_shift=-0.020, exponent=2.8),
            _section_yz(0.034, 0.072, 0.034, z_shift=-0.018, exponent=2.9),
            _section_yz(0.078, 0.084, 0.034, z_shift=-0.018, exponent=3.0),
            _section_yz(0.128, 0.066, 0.022, z_shift=-0.014, exponent=3.0),
            _section_yz(0.156, 0.040, 0.014, z_shift=-0.010, exponent=3.2),
        ],
    )

    hip_module = model.part("hip_module")
    hip_module.visual(hip_shell, material=matte_graphite, name="hip_shell")
    hip_module.visual(
        Box((0.086, hip_gap + 2.0 * hip_cheek_thickness, 0.014)),
        origin=Origin(xyz=(0.002, 0.0, 0.030)),
        material=satin_charcoal,
        name="hip_bridge",
    )
    hip_module.visual(
        Box((0.044, hip_cheek_thickness, 0.056)),
        origin=Origin(xyz=(0.004, hip_gap / 2.0 + hip_cheek_thickness / 2.0, -0.002)),
        material=satin_titanium,
        name="hip_left_cheek",
    )
    hip_module.visual(
        Box((0.044, hip_cheek_thickness, 0.056)),
        origin=Origin(xyz=(0.004, -hip_gap / 2.0 - hip_cheek_thickness / 2.0, -0.002)),
        material=satin_titanium,
        name="hip_right_cheek",
    )
    hip_module.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(
            xyz=(0.004, hip_gap / 2.0 + hip_cheek_thickness + 0.003, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_titanium,
        name="hip_left_axis_cap",
    )
    hip_module.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(
            xyz=(0.004, -hip_gap / 2.0 - hip_cheek_thickness - 0.003, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_titanium,
        name="hip_right_axis_cap",
    )
    hip_module.visual(
        Box((0.070, 0.048, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, 0.098)),
        material=dark_polymer,
        name="top_mount_plate",
    )
    hip_module.inertial = Inertial.from_geometry(
        Box((0.120, 0.100, 0.120)),
        mass=2.4,
        origin=Origin(xyz=(0.005, 0.0, 0.045)),
    )

    upper_leg = model.part("upper_leg")
    upper_leg.visual(
        Cylinder(radius=hip_barrel_radius, length=hip_barrel_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_charcoal,
        name="hip_barrel",
    )
    upper_leg.visual(
        Box((0.056, 0.050, 0.028)),
        origin=Origin(xyz=(0.002, 0.0, -0.010)),
        material=satin_charcoal,
        name="hip_interface_block",
    )
    upper_leg.visual(thigh_shell, material=matte_graphite, name="thigh_shell")
    upper_leg.visual(
        Box((0.060, 0.046, 0.012)),
        origin=Origin(xyz=(0.004, 0.0, -0.030)),
        material=satin_titanium,
        name="thigh_upper_band",
    )
    upper_leg.visual(
        Box((0.026, 0.036, 0.172)),
        origin=Origin(xyz=(0.004, 0.0, -0.112)),
        material=satin_charcoal,
        name="thigh_spine",
    )
    upper_leg.visual(
        Box((0.050, 0.044, 0.022)),
        origin=Origin(xyz=(0.004, 0.0, -0.188)),
        material=satin_charcoal,
        name="knee_interface_block",
    )
    upper_leg.visual(
        Box((0.036, knee_cheek_thickness, 0.042)),
        origin=Origin(
            xyz=(0.004, knee_gap / 2.0 + knee_cheek_thickness / 2.0, -0.215),
        ),
        material=satin_titanium,
        name="knee_left_cheek",
    )
    upper_leg.visual(
        Box((0.020, 0.016, 0.018)),
        origin=Origin(xyz=(0.004, 0.025, -0.198)),
        material=satin_titanium,
        name="knee_left_mount",
    )
    upper_leg.visual(
        Box((0.036, knee_cheek_thickness, 0.042)),
        origin=Origin(
            xyz=(0.004, -knee_gap / 2.0 - knee_cheek_thickness / 2.0, -0.215),
        ),
        material=satin_titanium,
        name="knee_right_cheek",
    )
    upper_leg.visual(
        Box((0.020, 0.016, 0.018)),
        origin=Origin(xyz=(0.004, -0.025, -0.198)),
        material=satin_titanium,
        name="knee_right_mount",
    )
    upper_leg.visual(
        Cylinder(radius=0.0175, length=0.012),
        origin=Origin(
            xyz=(0.004, knee_gap / 2.0 + knee_cheek_thickness - 0.001, -0.225),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_titanium,
        name="knee_left_axis_cap",
    )
    upper_leg.visual(
        Cylinder(radius=0.0175, length=0.012),
        origin=Origin(
            xyz=(0.004, -knee_gap / 2.0 - knee_cheek_thickness + 0.001, -0.225),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_titanium,
        name="knee_right_axis_cap",
    )
    upper_leg.visual(
        Box((0.008, 0.034, 0.092)),
        origin=Origin(xyz=(0.041, 0.0, -0.105)),
        material=satin_charcoal,
        name="upper_left_actuator_bay",
    )
    upper_leg.visual(
        Box((0.008, 0.034, 0.092)),
        origin=Origin(xyz=(-0.037, 0.0, -0.105)),
        material=satin_charcoal,
        name="upper_right_actuator_bay",
    )
    for bay_x, prefix in ((0.043, "upper_left"), (-0.039, "upper_right")):
        upper_leg.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(
                xyz=(bay_x, 0.0, -0.074),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_titanium,
            name=f"{prefix}_fastener_a",
        )
        upper_leg.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(
                xyz=(bay_x, 0.0, -0.138),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_titanium,
            name=f"{prefix}_fastener_b",
        )
    upper_leg.inertial = Inertial.from_geometry(
        Box((0.085, 0.080, 0.250)),
        mass=1.7,
        origin=Origin(xyz=(0.005, 0.0, -0.110)),
    )

    lower_leg = model.part("lower_leg")
    lower_leg.visual(
        Cylinder(radius=knee_barrel_radius, length=knee_barrel_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_charcoal,
        name="knee_barrel",
    )
    lower_leg.visual(
        Box((0.052, 0.044, 0.026)),
        origin=Origin(xyz=(0.006, 0.0, -0.010)),
        material=satin_charcoal,
        name="knee_interface_block",
    )
    lower_leg.visual(shin_shell, material=matte_graphite, name="shin_shell")
    lower_leg.visual(
        Box((0.056, 0.042, 0.012)),
        origin=Origin(xyz=(0.008, 0.0, -0.030)),
        material=satin_titanium,
        name="shin_upper_band",
    )
    lower_leg.visual(
        Box((0.022, 0.034, 0.174)),
        origin=Origin(xyz=(0.007, 0.0, -0.109)),
        material=satin_charcoal,
        name="shin_spine",
    )
    lower_leg.visual(
        Box((0.044, 0.040, 0.024)),
        origin=Origin(xyz=(0.006, 0.0, -0.190)),
        material=satin_charcoal,
        name="ankle_interface_block",
    )
    lower_leg.visual(
        Box((0.038, ankle_cheek_thickness, 0.040)),
        origin=Origin(
            xyz=(0.006, ankle_gap / 2.0 + ankle_cheek_thickness / 2.0, -0.220),
        ),
        material=satin_titanium,
        name="ankle_left_cheek",
    )
    lower_leg.visual(
        Box((0.010, 0.024, 0.028)),
        origin=Origin(xyz=(-0.014, 0.036, -0.198)),
        material=satin_titanium,
        name="ankle_left_mount",
    )
    lower_leg.visual(
        Box((0.038, ankle_cheek_thickness, 0.040)),
        origin=Origin(
            xyz=(0.006, -ankle_gap / 2.0 - ankle_cheek_thickness / 2.0, -0.220),
        ),
        material=satin_titanium,
        name="ankle_right_cheek",
    )
    lower_leg.visual(
        Box((0.010, 0.024, 0.028)),
        origin=Origin(xyz=(-0.014, -0.036, -0.198)),
        material=satin_titanium,
        name="ankle_right_mount",
    )
    lower_leg.visual(
        Cylinder(radius=0.0155, length=0.012),
        origin=Origin(
            xyz=(0.006, ankle_gap / 2.0 + ankle_cheek_thickness - 0.001, -0.220),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_titanium,
        name="ankle_left_axis_cap",
    )
    lower_leg.visual(
        Cylinder(radius=0.0155, length=0.012),
        origin=Origin(
            xyz=(0.006, -ankle_gap / 2.0 - ankle_cheek_thickness + 0.001, -0.220),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_titanium,
        name="ankle_right_axis_cap",
    )
    lower_leg.visual(
        Box((0.008, 0.032, 0.086)),
        origin=Origin(xyz=(0.039, 0.0, -0.108)),
        material=satin_charcoal,
        name="lower_left_actuator_bay",
    )
    lower_leg.visual(
        Box((0.008, 0.032, 0.086)),
        origin=Origin(xyz=(-0.033, 0.0, -0.108)),
        material=satin_charcoal,
        name="lower_right_actuator_bay",
    )
    for bay_x, prefix in ((0.041, "lower_left"), (-0.035, "lower_right")):
        lower_leg.visual(
            Cylinder(radius=0.0042, length=0.008),
            origin=Origin(
                xyz=(bay_x, 0.0, -0.080),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_titanium,
            name=f"{prefix}_fastener_a",
        )
        lower_leg.visual(
            Cylinder(radius=0.0042, length=0.008),
            origin=Origin(
                xyz=(bay_x, 0.0, -0.138),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_titanium,
            name=f"{prefix}_fastener_b",
        )
    lower_leg.inertial = Inertial.from_geometry(
        Box((0.080, 0.080, 0.245)),
        mass=1.5,
        origin=Origin(xyz=(0.008, 0.0, -0.112)),
    )

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=ankle_barrel_radius, length=ankle_barrel_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_charcoal,
        name="ankle_barrel",
    )
    foot.visual(
        Box((0.034, 0.056, 0.024)),
        origin=Origin(xyz=(0.000, 0.0, -0.010)),
        material=satin_charcoal,
        name="ankle_interface_block",
    )
    foot.visual(foot_shell, material=matte_graphite, name="foot_shell")
    foot.visual(
        Box((0.068, 0.056, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, -0.002)),
        material=satin_titanium,
        name="foot_upper_band",
    )
    foot.visual(
        Box((0.176, 0.080, 0.012)),
        origin=Origin(xyz=(0.048, 0.0, -0.043)),
        material=rubber_black,
        name="sole_pad",
    )
    foot.visual(
        Box((0.030, 0.070, 0.020)),
        origin=Origin(xyz=(-0.032, 0.0, -0.034)),
        material=dark_polymer,
        name="heel_bumper",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.190, 0.085, 0.070)),
        mass=0.9,
        origin=Origin(xyz=(0.048, 0.0, -0.026)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent="hip_module",
        child="upper_leg",
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=3.0,
            lower=-0.65,
            upper=1.10,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent="upper_leg",
        child="lower_leg",
        origin=Origin(xyz=(0.004, 0.0, -0.225)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=3.4,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent="lower_leg",
        child="foot",
        origin=Origin(xyz=(0.006, 0.0, -0.220)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=3.6,
            lower=-0.55,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.030)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_contact("hip_module", "upper_leg")
    ctx.expect_aabb_contact("upper_leg", "lower_leg")
    ctx.expect_aabb_contact("lower_leg", "foot")

    ctx.expect_aabb_overlap("hip_module", "upper_leg", axes="xy", min_overlap=0.040)
    ctx.expect_aabb_overlap("upper_leg", "lower_leg", axes="xy", min_overlap=0.032)
    ctx.expect_aabb_overlap("lower_leg", "foot", axes="xy", min_overlap=0.030)

    ctx.expect_aabb_gap(
        "hip_module",
        "upper_leg",
        axis="y",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="hip_left_cheek",
        negative_elem="hip_barrel",
        name="hip_left_axis_seat",
    )
    ctx.expect_aabb_gap(
        "upper_leg",
        "hip_module",
        axis="y",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="hip_barrel",
        negative_elem="hip_right_cheek",
        name="hip_right_axis_seat",
    )
    ctx.expect_aabb_gap(
        "upper_leg",
        "lower_leg",
        axis="y",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="knee_left_cheek",
        negative_elem="knee_barrel",
        name="knee_left_axis_seat",
    )
    ctx.expect_aabb_gap(
        "lower_leg",
        "upper_leg",
        axis="y",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="knee_barrel",
        negative_elem="knee_right_cheek",
        name="knee_right_axis_seat",
    )
    ctx.expect_aabb_gap(
        "lower_leg",
        "foot",
        axis="y",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="ankle_left_cheek",
        negative_elem="ankle_barrel",
        name="ankle_left_axis_seat",
    )
    ctx.expect_aabb_gap(
        "foot",
        "lower_leg",
        axis="y",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="ankle_barrel",
        negative_elem="ankle_right_cheek",
        name="ankle_right_axis_seat",
    )

    ctx.expect_joint_motion_axis(
        "hip_pitch",
        "upper_leg",
        world_axis="z",
        direction="positive",
        min_delta=0.025,
    )
    ctx.expect_joint_motion_axis(
        "knee_pitch",
        "lower_leg",
        world_axis="z",
        direction="positive",
        min_delta=0.035,
    )

    with ctx.pose(hip_pitch=0.75):
        ctx.expect_aabb_contact("hip_module", "upper_leg")

    with ctx.pose(knee_pitch=1.15):
        ctx.expect_aabb_contact("upper_leg", "lower_leg")
        ctx.expect_aabb_gap(
            "upper_leg",
            "lower_leg",
            axis="y",
            max_gap=0.001,
            max_penetration=0.001,
            positive_elem="knee_left_cheek",
            negative_elem="knee_barrel",
            name="knee_left_axis_seat_flexed",
        )

    with ctx.pose(ankle_pitch=-0.35):
        ctx.expect_aabb_contact("lower_leg", "foot")

    with ctx.pose(hip_pitch=0.50, knee_pitch=1.05, ankle_pitch=-0.20):
        ctx.expect_aabb_contact("hip_module", "upper_leg")
        ctx.expect_aabb_contact("upper_leg", "lower_leg")
        ctx.expect_aabb_contact("lower_leg", "foot")
        ctx.expect_aabb_gap(
            "lower_leg",
            "foot",
            axis="y",
            max_gap=0.001,
            max_penetration=0.001,
            positive_elem="ankle_left_cheek",
            negative_elem="ankle_barrel",
            name="ankle_left_axis_seat_loaded_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
