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
    BoxGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_off_utility_knife", assets=ASSETS)

    shell_orange = model.material("shell_orange", rgba=(0.94, 0.59, 0.13, 1.0))
    grip_charcoal = model.material("grip_charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    slider_black = model.material("slider_black", rgba=(0.11, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.73, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.48, 0.50, 0.54, 1.0))

    def yz_section(
        x_pos: float,
        *,
        width: float,
        height: float,
        radius: float,
        z_center: float | None = None,
    ) -> list[tuple[float, float, float]]:
        if z_center is None:
            z_center = height * 0.5
        profile = rounded_rect_profile(width, height, radius, corner_segments=5)
        return [(x_pos, y_pos, z_pos + z_center) for y_pos, z_pos in profile]

    def save_mesh(geometry, filename: str):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))

    def build_housing_shell():
        outer = repair_loft(
            section_loft(
                [
                    yz_section(0.000, width=0.034, height=0.018, radius=0.0036),
                    yz_section(0.112, width=0.034, height=0.018, radius=0.0036),
                    yz_section(0.148, width=0.024, height=0.015, radius=0.0028),
                    yz_section(0.175, width=0.015, height=0.012, radius=0.0022),
                ]
            )
        )
        inner = repair_loft(
            section_loft(
                [
                    yz_section(0.010, width=0.022, height=0.010, radius=0.0017, z_center=0.0072),
                    yz_section(0.118, width=0.022, height=0.010, radius=0.0017, z_center=0.0072),
                    yz_section(0.150, width=0.013, height=0.0088, radius=0.0013, z_center=0.0066),
                    yz_section(0.182, width=0.011, height=0.0084, radius=0.0011, z_center=0.0064),
                ]
            )
        )
        top_slot = BoxGeometry((0.080, 0.0068, 0.010)).translate(0.091, 0.0, 0.017)
        shell = boolean_difference(outer, inner)
        return boolean_difference(shell, top_slot)

    def build_blade_mesh():
        blade_profile = [
            (0.000, 0.000),
            (0.066, 0.000),
            (0.079, 0.0012),
            (0.092, 0.0044),
            (0.081, 0.0080),
            (0.000, 0.0080),
        ]
        blade = ExtrudeGeometry.centered(blade_profile, 0.0006).rotate_x(math.pi / 2.0)
        for x_pos in (0.018, 0.0275, 0.037, 0.0465):
            groove = BoxGeometry((0.0007, 0.0016, 0.0072)).translate(x_pos, 0.0, 0.0040)
            blade = boolean_difference(blade, groove)
        return blade

    def build_carrier_body_mesh():
        return repair_loft(
            section_loft(
                [
                    yz_section(0.002, width=0.012, height=0.0048, radius=0.0014, z_center=0.0054),
                    yz_section(0.048, width=0.012, height=0.0048, radius=0.0014, z_center=0.0054),
                    yz_section(0.064, width=0.011, height=0.0042, radius=0.0012, z_center=0.0052),
                    yz_section(0.076, width=0.0088, height=0.0032, radius=0.0010, z_center=0.0049),
                ]
            )
        )

    def build_blade_clamp_mesh():
        clamp_profile = [
            (0.000, 0.000),
            (0.016, 0.000),
            (0.020, 0.0012),
            (0.020, 0.0025),
            (0.008, 0.0038),
            (0.000, 0.0032),
        ]
        return ExtrudeGeometry.centered(clamp_profile, 0.0105).rotate_x(math.pi / 2.0)

    def build_thumb_slider_mesh():
        slider_profile = [
            (-0.0080, 0.0000),
            (-0.0062, 0.0018),
            (-0.0042, 0.0046),
            (-0.0016, 0.0034),
            (0.0000, 0.0059),
            (0.0016, 0.0034),
            (0.0042, 0.0046),
            (0.0062, 0.0018),
            (0.0080, 0.0000),
        ]
        return ExtrudeGeometry.centered(slider_profile, 0.0108).rotate_x(math.pi / 2.0)

    housing = model.part("housing")
    housing.visual(
        save_mesh(build_housing_shell(), "utility_knife_housing_shell.obj"),
        material=shell_orange,
        name="housing_shell",
    )
    housing.visual(
        Box((0.070, 0.0012, 0.0065)),
        origin=Origin(xyz=(0.078, 0.0176, 0.0072)),
        material=grip_charcoal,
        name="right_grip_pad",
    )
    housing.visual(
        Box((0.070, 0.0012, 0.0065)),
        origin=Origin(xyz=(0.078, -0.0176, 0.0072)),
        material=grip_charcoal,
        name="left_grip_pad",
    )
    housing.visual(
        Box((0.010, 0.027, 0.013)),
        origin=Origin(xyz=(0.005, 0.0, 0.0065)),
        material=grip_charcoal,
        name="rear_cap",
    )
    housing.visual(
        Box((0.112, 0.0016, 0.0032)),
        origin=Origin(xyz=(0.078, 0.0078, 0.0038)),
        material=grip_charcoal,
        name="right_guide",
    )
    housing.visual(
        Box((0.112, 0.0016, 0.0032)),
        origin=Origin(xyz=(0.078, -0.0078, 0.0038)),
        material=grip_charcoal,
        name="left_guide",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.175, 0.034, 0.018)),
        mass=0.42,
        origin=Origin(xyz=(0.0875, 0.0, 0.009)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.094, 0.011, 0.0032)),
        origin=Origin(xyz=(0.047, 0.0, 0.0016)),
        material=dark_steel,
        name="carrier_sled",
    )
    blade_carrier.visual(
        save_mesh(build_carrier_body_mesh(), "utility_knife_carrier_body.obj"),
        material=grip_charcoal,
        name="carrier_body",
    )
    blade_carrier.visual(
        save_mesh(build_blade_clamp_mesh(), "utility_knife_blade_clamp.obj"),
        origin=Origin(xyz=(0.063, 0.0, 0.0044)),
        material=grip_charcoal,
        name="blade_clamp",
    )
    blade_carrier.visual(
        Box((0.007, 0.0048, 0.0106)),
        origin=Origin(xyz=(0.050, 0.0, 0.0129)),
        material=slider_black,
        name="thumb_stem",
    )
    blade_carrier.visual(
        save_mesh(build_thumb_slider_mesh(), "utility_knife_thumb_slider.obj"),
        origin=Origin(xyz=(0.050, 0.0, 0.0169)),
        material=slider_black,
        name="thumb_slider",
    )
    blade_carrier.visual(
        save_mesh(build_blade_mesh(), "utility_knife_blade.obj"),
        origin=Origin(xyz=(0.068, 0.0, 0.0006)),
        material=steel,
        name="blade",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.110, 0.013, 0.022)),
        mass=0.12,
        origin=Origin(xyz=(0.055, 0.0, 0.011)),
    )

    model.articulation(
        "housing_to_blade_carrier",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=blade_carrier,
        origin=Origin(xyz=(0.018, 0.0, 0.0022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.15,
            lower=0.0,
            upper=0.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    blade_carrier = object_model.get_part("blade_carrier")
    slide = object_model.get_articulation("housing_to_blade_carrier")
    housing_shell = housing.get_visual("housing_shell")
    carrier_sled = blade_carrier.get_visual("carrier_sled")
    thumb_slider = blade_carrier.get_visual("thumb_slider")
    blade = blade_carrier.get_visual("blade")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=8)

    ctx.expect_contact(
        blade_carrier,
        housing,
        elem_a=carrier_sled,
        elem_b=housing_shell,
        contact_tol=5e-4,
        name="carrier sled rides on shell floor",
    )
    ctx.expect_overlap(
        blade_carrier,
        housing,
        axes="x",
        elem_a=carrier_sled,
        elem_b=housing_shell,
        min_overlap=0.085,
        name="carrier remains deeply captured in housing at rest",
    )
    ctx.expect_within(
        blade_carrier,
        housing,
        axes="y",
        inner_elem=carrier_sled,
        outer_elem=housing_shell,
        margin=0.012,
        name="carrier stays centered within housing width",
    )

    def element_aabb(part, elem, label: str):
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            ctx.fail(f"{label} measurable", "Missing world-space element AABB.")
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        return box

    def part_pos(part, label: str):
        pos = ctx.part_world_position(part)
        if pos is None:
            ctx.fail(f"{label} measurable", "Missing world-space part position.")
            return (0.0, 0.0, 0.0)
        return pos

    housing_box = element_aabb(housing, housing_shell, "housing shell")
    blade_rest_box = element_aabb(blade_carrier, blade, "rest blade")
    slider_rest_box = element_aabb(blade_carrier, thumb_slider, "rest thumb slider")
    rest_pos = part_pos(blade_carrier, "rest carrier")

    rest_exposure = blade_rest_box[1][0] - housing_box[1][0]
    slider_rest_center_y = 0.5 * (slider_rest_box[0][1] + slider_rest_box[1][1])
    ctx.check(
        "blade slightly exposed at rest",
        0.002 <= rest_exposure <= 0.008,
        details=f"Expected 2-8 mm rest exposure, got {rest_exposure:.4f} m.",
    )
    ctx.check(
        "thumb slider sits on top centerline",
        abs(slider_rest_center_y) <= 0.0015 and slider_rest_box[1][2] > housing_box[1][2] + 0.003,
        details=(
            f"Slider center y={slider_rest_center_y:.4f} m, "
            f"slider top z={slider_rest_box[1][2]:.4f} m, housing top z={housing_box[1][2]:.4f} m."
        ),
    )

    with ctx.pose({slide: 0.025}):
        mid_pos = part_pos(blade_carrier, "mid carrier")
        blade_mid_box = element_aabb(blade_carrier, blade, "mid blade")
        mid_exposure = blade_mid_box[1][0] - housing_box[1][0]
        ctx.expect_contact(
            blade_carrier,
            housing,
            elem_a=carrier_sled,
            elem_b=housing_shell,
            contact_tol=5e-4,
            name="carrier remains supported at mid extension",
        )
        ctx.check(
            "mid extension gives intermediate blade reveal",
            0.024 <= mid_exposure <= 0.031,
            details=f"Expected about 25 mm mid exposure, got {mid_exposure:.4f} m.",
        )

    with ctx.pose({slide: 0.05}):
        full_pos = part_pos(blade_carrier, "full carrier")
        blade_full_box = element_aabb(blade_carrier, blade, "full blade")
        slider_full_box = element_aabb(blade_carrier, thumb_slider, "full thumb slider")
        full_exposure = blade_full_box[1][0] - housing_box[1][0]
        ctx.expect_contact(
            blade_carrier,
            housing,
            elem_a=carrier_sled,
            elem_b=housing_shell,
            contact_tol=5e-4,
            name="carrier remains supported at full extension",
        )
        ctx.expect_overlap(
            blade_carrier,
            housing,
            axes="x",
            elem_a=carrier_sled,
            elem_b=housing_shell,
            min_overlap=0.035,
            name="carrier remains captured at full extension",
        )
        ctx.check(
            "full extension exposes about 50 mm of blade",
            0.050 <= full_exposure <= 0.057,
            details=f"Expected roughly 50-57 mm full exposure, got {full_exposure:.4f} m.",
        )
        ctx.check(
            "thumb slider stays inside top travel window",
            slider_full_box[1][0] <= housing_box[1][0] - 0.045 and slider_full_box[0][0] >= 0.056,
            details=(
                f"Slider x-range=({slider_full_box[0][0]:.4f}, {slider_full_box[1][0]:.4f}) m, "
                f"housing nose max x={housing_box[1][0]:.4f} m."
            ),
        )

    ctx.check(
        "carrier translates straight along knife axis",
        abs((mid_pos[0] - rest_pos[0]) - 0.025) <= 5e-4
        and abs((full_pos[0] - rest_pos[0]) - 0.05) <= 5e-4
        and abs(mid_pos[1] - rest_pos[1]) <= 1e-6
        and abs(full_pos[1] - rest_pos[1]) <= 1e-6
        and abs(mid_pos[2] - rest_pos[2]) <= 1e-6
        and abs(full_pos[2] - rest_pos[2]) <= 1e-6,
        details=(
            f"rest={rest_pos}, mid={mid_pos}, full={full_pos}; "
            "expected pure 25 mm and 50 mm x translation with no y/z drift."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
