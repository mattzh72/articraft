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


def box_solid(
    sx: float,
    sy: float,
    sz: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z_min: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz).translate((x, y, z_min + sz / 2.0))


def channel_solid(
    height: float,
    outer_w: float,
    depth: float,
    web_t: float,
    flange_t: float,
    *,
    open_dir: int,
) -> cq.Workplane:
    outer = box_solid(outer_w, depth, height, z_min=0.0)
    inner = box_solid(
        outer_w - web_t,
        depth - 2.0 * flange_t,
        height + 0.004,
        x=open_dir * (web_t / 2.0),
        z_min=-0.002,
    )
    return outer.cut(inner)


def guide_shoe(x_center: float, z_center: float) -> cq.Workplane:
    outer = box_solid(0.050, 0.041, 0.086, x=x_center, y=0.0355, z_min=z_center - 0.043)
    cavity = box_solid(0.034, 0.050, 0.090, x=x_center, y=0.0200, z_min=z_center - 0.045)
    return outer.cut(cavity)


def roller_x(
    length: float,
    radius: float,
    *,
    x_center: float,
    y_center: float,
    z_center: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((x_center - length / 2.0, y_center, z_center))
    )


def fork_assembly(x_center: float) -> cq.Workplane:
    tine = (
        box_solid(0.095, 0.580, 0.042, x=x_center, y=0.380, z_min=0.000)
        .edges(">Y and <Z")
        .chamfer(0.026)
        .edges(">Y and >Z")
        .chamfer(0.006)
    )
    heel = box_solid(0.095, 0.060, 0.060, x=x_center, y=0.120, z_min=0.000)
    shank = box_solid(0.095, 0.025, 0.320, x=x_center, y=0.086, z_min=0.020)
    upper_hook = box_solid(0.095, 0.048, 0.030, x=x_center, y=0.066, z_min=0.304)
    lower_tab = box_solid(0.080, 0.038, 0.024, x=x_center, y=0.066, z_min=0.120)
    return tine.union(heel).union(shank).union(upper_hook).union(lower_tab)


def build_mast() -> cq.Workplane:
    left_rail = channel_solid(
        1.360,
        0.078,
        0.100,
        0.016,
        0.011,
        open_dir=1,
    ).translate((-0.230, -0.005, 0.120))
    right_rail = channel_solid(
        1.360,
        0.078,
        0.100,
        0.016,
        0.011,
        open_dir=-1,
    ).translate((0.230, -0.005, 0.120))

    left_track = box_solid(0.032, 0.014, 1.160, x=-0.207, y=0.038, z_min=0.180)
    right_track = box_solid(0.032, 0.014, 1.160, x=0.207, y=0.038, z_min=0.180)

    base_tie = box_solid(0.560, 0.085, 0.120, y=-0.015, z_min=0.000)
    top_tie = box_solid(0.560, 0.070, 0.055, y=-0.015, z_min=1.485)
    rear_bridge = box_solid(0.400, 0.012, 0.520, y=-0.048, z_min=0.350)
    upper_guard = box_solid(0.460, 0.018, 0.030, y=0.010, z_min=1.420)

    mast = left_rail.union(right_rail)
    mast = mast.union(left_track).union(right_track)
    mast = mast.union(base_tie).union(top_tie).union(rear_bridge).union(upper_guard)
    return mast


def build_carriage_body() -> cq.Workplane:
    left_upright = channel_solid(
        0.360,
        0.088,
        0.056,
        0.012,
        0.010,
        open_dir=1,
    ).translate((-0.205, 0.083, 0.000))
    right_upright = channel_solid(
        0.360,
        0.088,
        0.056,
        0.012,
        0.010,
        open_dir=-1,
    ).translate((0.205, 0.083, 0.000))

    lower_bar = box_solid(0.560, 0.030, 0.045, y=0.083, z_min=0.100)
    upper_bar = box_solid(0.560, 0.038, 0.055, y=0.083, z_min=0.300)

    center_plate = box_solid(0.260, 0.012, 0.180, y=0.064, z_min=0.120)
    rib_left = box_solid(0.016, 0.022, 0.170, x=-0.140, y=0.067, z_min=0.125)
    rib_center = box_solid(0.018, 0.022, 0.170, x=0.000, y=0.067, z_min=0.125)
    rib_right = box_solid(0.016, 0.022, 0.170, x=0.140, y=0.067, z_min=0.125)
    side_gusset_left = box_solid(0.026, 0.020, 0.160, x=-0.175, y=0.068, z_min=0.120)
    side_gusset_right = box_solid(0.026, 0.020, 0.160, x=0.175, y=0.068, z_min=0.120)

    top_wear_left = box_solid(0.120, 0.006, 0.032, x=-0.145, y=0.107, z_min=0.311)
    top_wear_right = box_solid(0.120, 0.006, 0.032, x=0.145, y=0.107, z_min=0.311)
    lower_wear_left = box_solid(0.100, 0.006, 0.024, x=-0.145, y=0.106, z_min=0.118)
    lower_wear_right = box_solid(0.100, 0.006, 0.024, x=0.145, y=0.106, z_min=0.118)

    body = left_upright.union(right_upright)
    body = body.union(lower_bar).union(upper_bar)
    body = body.union(center_plate)
    body = body.union(rib_left).union(rib_center).union(rib_right)
    body = body.union(side_gusset_left).union(side_gusset_right)
    body = body.union(top_wear_left).union(top_wear_right)
    body = body.union(lower_wear_left).union(lower_wear_right)
    return body


def build_guide_hardware() -> cq.Workplane:
    hardware: cq.Workplane | None = None
    for x_center in (-0.207, 0.207):
        for z_center in (0.110, 0.290):
            bracket = box_solid(0.032, 0.018, 0.050, x=x_center, y=0.062, z_min=z_center - 0.025)
            axle_boss = box_solid(0.014, 0.020, 0.020, x=x_center, y=0.070, z_min=z_center - 0.010)
            front_roller = roller_x(
                0.026,
                0.012,
                x_center=x_center,
                y_center=0.057,
                z_center=z_center,
            )
            keeper_tab_x = x_center - 0.022 if x_center < 0.0 else x_center + 0.022
            keeper_tab = box_solid(
                0.012,
                0.014,
                0.046,
                x=keeper_tab_x,
                y=0.059,
                z_min=z_center - 0.023,
            )
            assembly = bracket.union(axle_boss).union(front_roller).union(keeper_tab)
            hardware = assembly if hardware is None else hardware.union(assembly)
    assert hardware is not None
    return hardware


def build_backrest() -> cq.Workplane:
    left_post = box_solid(0.020, 0.014, 0.460, x=-0.190, y=0.083, z_min=0.350)
    right_post = box_solid(0.020, 0.014, 0.460, x=0.190, y=0.083, z_min=0.350)
    center_post = box_solid(0.016, 0.012, 0.410, x=0.000, y=0.083, z_min=0.370)
    upper_bar = box_solid(0.460, 0.014, 0.024, y=0.083, z_min=0.786)
    slat_low = box_solid(0.420, 0.010, 0.018, y=0.083, z_min=0.432)
    slat_mid_low = box_solid(0.420, 0.010, 0.018, y=0.083, z_min=0.542)
    slat_mid_high = box_solid(0.420, 0.010, 0.018, y=0.083, z_min=0.652)
    slat_high = box_solid(0.420, 0.010, 0.018, y=0.083, z_min=0.752)

    backrest = left_post.union(right_post).union(center_post)
    backrest = backrest.union(upper_bar)
    backrest = backrest.union(slat_low).union(slat_mid_low).union(slat_mid_high).union(slat_high)
    return backrest


def build_forks() -> cq.Workplane:
    return fork_assembly(-0.145).union(fork_assembly(0.145)).translate((0.0, 0.015, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_carriage_module")

    mast_color = model.material("mast_charcoal", rgba=(0.23, 0.25, 0.28, 1.0))
    carriage_color = model.material("carriage_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    backrest_color = model.material("backrest_gray", rgba=(0.36, 0.37, 0.39, 1.0))
    fork_color = model.material("fork_steel", rgba=(0.17, 0.17, 0.18, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(build_mast(), "mast_frame"),
        origin=Origin(),
        material=mast_color,
        name="mast_frame",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(build_carriage_body(), "carriage_body"),
        origin=Origin(),
        material=carriage_color,
        name="carriage_body",
    )
    carriage.visual(
        mesh_from_cadquery(build_guide_hardware(), "guide_hardware"),
        origin=Origin(),
        material=fork_color,
        name="guide_hardware",
    )
    carriage.visual(
        mesh_from_cadquery(build_backrest(), "load_backrest"),
        origin=Origin(),
        material=backrest_color,
        name="load_backrest",
    )
    carriage.visual(
        mesh_from_cadquery(build_forks(), "fork_pair"),
        origin=Origin(),
        material=fork_color,
        name="fork_pair",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15000.0,
            velocity=0.60,
            lower=0.0,
            upper=0.72,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")
    mast_frame = mast.get_visual("mast_frame")
    carriage_body = carriage.get_visual("carriage_body")
    guide_hardware = carriage.get_visual("guide_hardware")
    fork_pair = carriage.get_visual("fork_pair")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
    ctx.expect_contact(
        carriage,
        mast,
        elem_a=guide_hardware,
        elem_b=mast_frame,
        contact_tol=0.002,
        name="guide_rollers_support_carriage_on_mast_tracks_at_rest",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="y",
        positive_elem=carriage_body,
        negative_elem=mast_frame,
        min_gap=0.008,
        name="carriage_frame_clears_mast_structure",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="y",
        positive_elem=fork_pair,
        negative_elem=mast_frame,
        min_gap=0.008,
        name="fork_tabs_and_hooks_clear_mast",
    )
    ctx.expect_within(
        carriage,
        mast,
        axes="x",
        margin=0.055,
        name="carriage_stays_between_mast_rails",
    )

    with ctx.pose({lift: 0.72}):
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=guide_hardware,
            elem_b=mast_frame,
            contact_tol=0.002,
            name="guide_rollers_remain_supported_at_full_raise",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="y",
            positive_elem=carriage_body,
            negative_elem=mast_frame,
            min_gap=0.008,
            name="carriage_frame_clears_mast_at_full_raise",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_raise")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
