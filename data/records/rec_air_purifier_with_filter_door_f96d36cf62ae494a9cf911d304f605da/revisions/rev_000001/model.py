from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_panel_air_purifier")

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.93, 1.0))
    trim_white = model.material("trim_white", rgba=(0.88, 0.89, 0.87, 1.0))
    vent_grey = model.material("vent_grey", rgba=(0.78, 0.80, 0.80, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.24, 0.25, 1.0))
    filter_fiber = model.material("filter_fiber", rgba=(0.56, 0.66, 0.64, 1.0))
    latch_grey = model.material("latch_grey", rgba=(0.62, 0.64, 0.64, 1.0))

    housing_w = 0.56
    housing_h = 0.82
    housing_d = 0.105
    back_t = 0.004
    side_t = 0.018
    top_t = 0.018
    bottom_t = 0.024
    bezel_d = 0.012
    bezel_w = 0.018
    open_w = housing_w - 2.0 * bezel_w
    open_h = housing_h - (top_t + bottom_t)

    panel_w = 0.524
    panel_h = 0.786
    panel_skin_t = 0.013
    panel_feature_t = 0.004

    filter_w = 0.500
    filter_h = 0.710
    filter_d = 0.078
    frame_strip = 0.022
    media_w = 0.456
    media_h = 0.662
    media_d = 0.058

    rail_len = 0.070
    rail_w = 0.022
    rail_h = 0.014
    rail_y = 0.198
    rail_top_z = -housing_h / 2.0 + bottom_t + rail_h
    rail_center_x = 0.055
    rail_center_z = rail_top_z - rail_h / 2.0

    housing = model.part("housing")
    housing.visual(
        Box((back_t, housing_w, housing_h)),
        origin=Origin(xyz=(back_t / 2.0, 0.0, 0.0)),
        material=body_white,
        name="back_plate",
    )
    housing.visual(
        Box((housing_d, side_t, housing_h)),
        origin=Origin(xyz=(housing_d / 2.0, -(housing_w - side_t) / 2.0, 0.0)),
        material=body_white,
        name="left_wall",
    )
    housing.visual(
        Box((housing_d, side_t, housing_h)),
        origin=Origin(xyz=(housing_d / 2.0, (housing_w - side_t) / 2.0, 0.0)),
        material=body_white,
        name="right_wall",
    )
    housing.visual(
        Box((housing_d, housing_w - 2.0 * side_t, top_t)),
        origin=Origin(xyz=(housing_d / 2.0, 0.0, housing_h / 2.0 - top_t / 2.0)),
        material=body_white,
        name="top_wall",
    )
    housing.visual(
        Box((housing_d, housing_w - 2.0 * side_t, bottom_t)),
        origin=Origin(xyz=(housing_d / 2.0, 0.0, -housing_h / 2.0 + bottom_t / 2.0)),
        material=body_white,
        name="bottom_wall",
    )
    housing.visual(
        Box((bezel_d, open_w, top_t)),
        origin=Origin(xyz=(housing_d - bezel_d / 2.0, 0.0, housing_h / 2.0 - top_t / 2.0)),
        material=trim_white,
        name="front_bezel_top",
    )
    housing.visual(
        Box((bezel_d, open_w, bottom_t)),
        origin=Origin(xyz=(housing_d - bezel_d / 2.0, 0.0, -housing_h / 2.0 + bottom_t / 2.0)),
        material=trim_white,
        name="front_bezel_bottom",
    )
    housing.visual(
        Box((bezel_d, bezel_w, open_h)),
        origin=Origin(
            xyz=(housing_d - bezel_d / 2.0, -(housing_w - bezel_w) / 2.0, (top_t - bottom_t) / 2.0)
        ),
        material=trim_white,
        name="front_bezel_left",
    )
    housing.visual(
        Box((bezel_d, bezel_w, open_h)),
        origin=Origin(
            xyz=(housing_d - bezel_d / 2.0, (housing_w - bezel_w) / 2.0, (top_t - bottom_t) / 2.0)
        ),
        material=trim_white,
        name="front_bezel_right",
    )
    housing.visual(
        Box((0.018, 0.060, 0.012)),
        origin=Origin(xyz=(housing_d - 0.009, 0.0, housing_h / 2.0 - 0.013)),
        material=latch_grey,
        name="status_light_bar",
    )
    housing.visual(
        Box((rail_len, rail_w, rail_h)),
        origin=Origin(xyz=(rail_center_x, -rail_y, rail_center_z)),
        material=charcoal,
        name="rail_left",
    )
    housing.visual(
        Box((rail_len, rail_w, rail_h)),
        origin=Origin(xyz=(rail_center_x, rail_y, rail_center_z)),
        material=charcoal,
        name="rail_right",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_d, housing_w, housing_h)),
        mass=8.5,
        origin=Origin(xyz=(housing_d / 2.0, 0.0, 0.0)),
    )

    filter_panel = model.part("filter_panel")
    filter_panel.visual(
        Box((panel_skin_t, panel_w, panel_h)),
        origin=Origin(xyz=(panel_skin_t / 2.0, panel_w / 2.0, 0.0)),
        material=body_white,
        name="panel_skin",
    )
    filter_panel.visual(
        Box((0.010, panel_w - 0.050, panel_h - 0.070)),
        origin=Origin(xyz=(0.008, panel_w / 2.0, 0.0)),
        material=trim_white,
        name="panel_inner_field",
    )
    slat_count = 10
    slat_pitch = (panel_h - 0.180) / (slat_count - 1)
    for index in range(slat_count):
        z_pos = -panel_h / 2.0 + 0.090 + index * slat_pitch
        filter_panel.visual(
            Box((panel_feature_t, panel_w - 0.080, 0.014)),
            origin=Origin(xyz=(panel_skin_t + panel_feature_t / 2.0 - 0.001, panel_w / 2.0, z_pos)),
            material=vent_grey,
            name=f"intake_slat_{index}",
        )
    filter_panel.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(
            xyz=(panel_skin_t + 0.010, panel_w - 0.018, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=latch_grey,
        name="pull_grip",
    )
    filter_panel.inertial = Inertial.from_geometry(
        Box((0.022, panel_w, panel_h)),
        mass=1.4,
        origin=Origin(xyz=(0.011, panel_w / 2.0, 0.0)),
    )

    filter_cartridge = model.part("filter_cartridge")
    filter_cartridge.visual(
        Box((filter_d, frame_strip, filter_h)),
        origin=Origin(xyz=(-filter_d / 2.0, -(filter_w - frame_strip) / 2.0, 0.0)),
        material=charcoal,
        name="frame_left",
    )
    filter_cartridge.visual(
        Box((filter_d, frame_strip, filter_h)),
        origin=Origin(xyz=(-filter_d / 2.0, (filter_w - frame_strip) / 2.0, 0.0)),
        material=charcoal,
        name="frame_right",
    )
    filter_cartridge.visual(
        Box((filter_d, filter_w, frame_strip)),
        origin=Origin(xyz=(-filter_d / 2.0, 0.0, filter_h / 2.0 - frame_strip / 2.0)),
        material=charcoal,
        name="frame_top",
    )
    filter_cartridge.visual(
        Box((filter_d, filter_w, frame_strip)),
        origin=Origin(xyz=(-filter_d / 2.0, 0.0, -filter_h / 2.0 + frame_strip / 2.0)),
        material=charcoal,
        name="frame_bottom",
    )
    filter_cartridge.visual(
        Box((media_d, media_w, media_h)),
        origin=Origin(xyz=(-filter_d / 2.0, 0.0, 0.0)),
        material=filter_fiber,
        name="filter_media",
    )
    pleat_count = 6
    pleat_pitch = media_w / (pleat_count + 1)
    for index in range(pleat_count):
        y_pos = -media_w / 2.0 + (index + 1) * pleat_pitch
        filter_cartridge.visual(
            Box((0.010, 0.010, media_h - 0.040)),
            origin=Origin(xyz=(-filter_d / 2.0 + media_d / 2.0 - 0.006, y_pos, 0.0)),
            material=vent_grey,
            name=f"pleat_rib_{index}",
        )
    filter_cartridge.visual(
        Box((0.010, 0.140, 0.030)),
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
        material=latch_grey,
        name="cartridge_pull_tab",
    )
    filter_cartridge.visual(
        Box((0.024, 0.050, 0.016)),
        origin=Origin(xyz=(-0.030, -rail_y, -0.359)),
        material=charcoal,
        name="runner_mount_left",
    )
    filter_cartridge.visual(
        Box((0.024, 0.050, 0.016)),
        origin=Origin(xyz=(-0.030, rail_y, -0.359)),
        material=charcoal,
        name="runner_mount_right",
    )
    filter_cartridge.visual(
        Box((rail_len, rail_w, 0.006)),
        origin=Origin(xyz=(-0.036, -rail_y, -0.369)),
        material=charcoal,
        name="skid_left",
    )
    filter_cartridge.visual(
        Box((rail_len, rail_w, 0.006)),
        origin=Origin(xyz=(-0.036, rail_y, -0.369)),
        material=charcoal,
        name="skid_right",
    )
    filter_cartridge.inertial = Inertial.from_geometry(
        Box((filter_d, filter_w, filter_h)),
        mass=1.9,
        origin=Origin(xyz=(-filter_d / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_filter_panel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=filter_panel,
        origin=Origin(xyz=(housing_d, -panel_w / 2.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "housing_to_filter_cartridge",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_cartridge,
        origin=Origin(xyz=(0.091, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.12,
            lower=0.0,
            upper=0.055,
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

    housing = object_model.get_part("housing")
    filter_panel = object_model.get_part("filter_panel")
    filter_cartridge = object_model.get_part("filter_cartridge")
    panel_hinge = object_model.get_articulation("housing_to_filter_panel")
    cartridge_slide = object_model.get_articulation("housing_to_filter_cartridge")

    with ctx.pose({panel_hinge: 0.0, cartridge_slide: 0.0}):
        ctx.expect_gap(
            filter_panel,
            housing,
            axis="x",
            min_gap=0.0,
            max_gap=0.001,
            name="closed panel sits flush on the housing front",
        )
        ctx.expect_overlap(
            filter_panel,
            housing,
            axes="yz",
            min_overlap=0.50,
            name="closed panel covers the front opening footprint",
        )
        ctx.expect_gap(
            filter_panel,
            filter_cartridge,
            axis="x",
            min_gap=0.010,
            max_gap=0.020,
            name="closed panel clears the seated filter cartridge",
        )
        ctx.expect_within(
            filter_cartridge,
            housing,
            axes="yz",
            margin=0.0,
            name="seated filter cartridge stays inside the housing aperture",
        )
        ctx.expect_overlap(
            filter_cartridge,
            housing,
            axes="x",
            min_overlap=0.075,
            name="seated filter cartridge remains deeply inserted",
        )
        ctx.expect_contact(
            filter_cartridge,
            housing,
            elem_a="skid_left",
            elem_b="rail_left",
            name="left guide skid rests on the left rail at rest",
        )
        ctx.expect_contact(
            filter_cartridge,
            housing,
            elem_a="skid_right",
            elem_b="rail_right",
            name="right guide skid rests on the right rail at rest",
        )
        closed_panel_aabb = ctx.part_world_aabb(filter_panel)
        seated_filter_pos = ctx.part_world_position(filter_cartridge)

    with ctx.pose({panel_hinge: math.radians(95.0), cartridge_slide: 0.055}):
        ctx.expect_within(
            filter_cartridge,
            housing,
            axes="yz",
            margin=0.0,
            name="extended filter cartridge stays aligned on the housing rails",
        )
        ctx.expect_overlap(
            filter_cartridge,
            housing,
            axes="x",
            min_overlap=0.035,
            name="extended filter cartridge retains insertion in the housing",
        )
        ctx.expect_contact(
            filter_cartridge,
            housing,
            elem_a="skid_left",
            elem_b="rail_left",
            name="left guide skid stays supported when the cartridge is extended",
        )
        ctx.expect_contact(
            filter_cartridge,
            housing,
            elem_a="skid_right",
            elem_b="rail_right",
            name="right guide skid stays supported when the cartridge is extended",
        )
        open_panel_aabb = ctx.part_world_aabb(filter_panel)
        extended_filter_pos = ctx.part_world_position(filter_cartridge)

    panel_swings_outward = (
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][0] > closed_panel_aabb[1][0] + 0.20
    )
    ctx.check(
        "panel hinge swings the front door outward",
        panel_swings_outward,
        details=f"closed_aabb={closed_panel_aabb}, open_aabb={open_panel_aabb}",
    )

    cartridge_extends_forward = (
        seated_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[0] > seated_filter_pos[0] + 0.045
    )
    ctx.check(
        "filter cartridge slides forward out of the housing",
        cartridge_extends_forward,
        details=f"rest={seated_filter_pos}, extended={extended_filter_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
