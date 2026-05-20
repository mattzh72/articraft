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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _arc_points(
    center: tuple[float, float],
    radius: float,
    start_angle: float,
    end_angle: float,
    segments: int,
) -> list[tuple[float, float]]:
    return [
        (
            center[0] + radius * math.cos(start_angle + (end_angle - start_angle) * step / segments),
            center[1] + radius * math.sin(start_angle + (end_angle - start_angle) * step / segments),
        )
        for step in range(segments + 1)
    ]


def _rounded_notched_profile(
    *,
    half_width: float,
    rear_y: float,
    front_y: float,
    notch_half_width: float,
    notch_back_y: float,
    corner_radius: float,
    arc_segments: int = 8,
) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = [(-notch_half_width, front_y), (-half_width + corner_radius, front_y)]
    profile.extend(
        _arc_points(
            (-half_width + corner_radius, front_y - corner_radius),
            corner_radius,
            math.pi / 2.0,
            math.pi,
            arc_segments,
        )[1:]
    )
    profile.append((-half_width, rear_y + corner_radius))
    profile.extend(
        _arc_points(
            (-half_width + corner_radius, rear_y + corner_radius),
            corner_radius,
            math.pi,
            3.0 * math.pi / 2.0,
            arc_segments,
        )[1:]
    )
    profile.append((half_width - corner_radius, rear_y))
    profile.extend(
        _arc_points(
            (half_width - corner_radius, rear_y + corner_radius),
            corner_radius,
            3.0 * math.pi / 2.0,
            2.0 * math.pi,
            arc_segments,
        )[1:]
    )
    profile.append((half_width, front_y - corner_radius))
    profile.extend(
        _arc_points(
            (half_width - corner_radius, front_y - corner_radius),
            corner_radius,
            0.0,
            math.pi / 2.0,
            arc_segments,
        )[1:]
    )
    profile.extend(
        [
            (notch_half_width, front_y),
            (notch_half_width, notch_back_y),
            (-notch_half_width, notch_back_y),
        ]
    )
    return profile


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[index] + hi[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_globe_air_purifier")

    warm_white = model.material("warm_white", rgba=(0.93, 0.94, 0.95, 1.0))
    satin_white = model.material("satin_white", rgba=(0.89, 0.90, 0.91, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.25, 0.27, 0.29, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.49, 0.53, 0.56, 1.0))
    soft_black = model.material("soft_black", rgba=(0.09, 0.10, 0.11, 1.0))

    base_outer = _rounded_notched_profile(
        half_width=0.155,
        rear_y=-0.150,
        front_y=0.150,
        notch_half_width=0.078,
        notch_back_y=0.050,
        corner_radius=0.050,
    )
    base_inner = list(
        reversed(
            _rounded_notched_profile(
                half_width=0.128,
                rear_y=-0.123,
                front_y=0.123,
                notch_half_width=0.066,
                notch_back_y=0.050,
                corner_radius=0.040,
            )
        )
    )
    base_skirt_mesh = _save_mesh(
        "base_skirt",
        ExtrudeWithHolesGeometry(
            base_outer,
            [base_inner],
            height=0.068,
            center=True,
        ).translate(0.0, 0.0, 0.044),
    )
    upper_intake_band_mesh = _save_mesh(
        "upper_intake_band",
        TorusGeometry(radius=0.121, tube=0.010, radial_segments=18, tubular_segments=56),
    )
    lower_intake_band_mesh = _save_mesh(
        "lower_intake_band",
        TorusGeometry(radius=0.104, tube=0.010, radial_segments=18, tubular_segments=48),
    )

    lower_shell_outer = [
        (0.050, -0.105),
        (0.078, -0.098),
        (0.104, -0.082),
        (0.126, -0.055),
        (0.137, -0.025),
        (0.140, 0.000),
    ]
    lower_shell_inner = [
        (0.041, -0.101),
        (0.067, -0.094),
        (0.091, -0.079),
        (0.114, -0.054),
        (0.126, -0.024),
        (0.129, 0.000),
    ]
    upper_shell_outer = [
        (0.140, 0.000),
        (0.138, 0.028),
        (0.129, 0.065),
        (0.108, 0.102),
        (0.070, 0.129),
        (0.024, 0.138),
        (0.012, 0.140),
    ]
    upper_shell_inner = [
        (0.129, 0.000),
        (0.127, 0.027),
        (0.119, 0.060),
        (0.101, 0.093),
        (0.066, 0.117),
        (0.023, 0.126),
        (0.010, 0.129),
    ]

    lower_shell_mesh = _save_mesh(
        "lower_shell",
        LatheGeometry.from_shell_profiles(
            lower_shell_outer,
            lower_shell_inner,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    upper_shell_mesh = _save_mesh(
        "upper_shell",
        LatheGeometry.from_shell_profiles(
            upper_shell_outer,
            upper_shell_inner,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.155, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=charcoal,
        name="bottom_disc",
    )
    base.visual(
        Box((0.220, 0.082, 0.068)),
        origin=Origin(xyz=(0.0, -0.106, 0.044)),
        material=graphite,
        name="base_skirt",
    )
    base.visual(
        Box((0.034, 0.166, 0.068)),
        origin=Origin(xyz=(-0.111, -0.010, 0.044)),
        material=graphite,
        name="left_skirt_wall",
    )
    base.visual(
        Box((0.034, 0.166, 0.068)),
        origin=Origin(xyz=(0.111, -0.010, 0.044)),
        material=graphite,
        name="right_skirt_wall",
    )
    base.visual(
        Box((0.034, 0.036, 0.068)),
        origin=Origin(xyz=(-0.111, 0.118, 0.044)),
        material=graphite,
        name="left_front_cheek",
    )
    base.visual(
        Box((0.034, 0.036, 0.068)),
        origin=Origin(xyz=(0.111, 0.118, 0.044)),
        material=graphite,
        name="right_front_cheek",
    )
    base.visual(
        Cylinder(radius=0.155, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=charcoal,
        name="top_plate",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=graphite,
        name="neck_pad",
    )
    base.visual(
        Box((0.012, 0.004, 0.068)),
        origin=Origin(xyz=(-0.075, 0.153, 0.044)),
        material=charcoal,
        name="slot_jamb_left",
    )
    base.visual(
        Box((0.012, 0.004, 0.068)),
        origin=Origin(xyz=(0.075, 0.153, 0.044)),
        material=charcoal,
        name="slot_jamb_right",
    )
    base.visual(
        Box((0.138, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.153, 0.076)),
        material=charcoal,
        name="slot_header",
    )
    base.visual(
        Box((0.138, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.153, 0.015)),
        material=charcoal,
        name="slot_sill",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.310, 0.310, 0.100)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    lower_housing = model.part("lower_housing")
    lower_housing.visual(lower_shell_mesh, material=warm_white, name="lower_shell")
    lower_housing.visual(
        Cylinder(radius=0.042, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, -0.138)),
        material=satin_white,
        name="support_neck",
    )
    lower_housing.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=warm_white,
        name="neck_collar",
    )
    lower_housing.visual(
        upper_intake_band_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=graphite,
        name="intake_band_upper",
    )
    lower_housing.visual(
        lower_intake_band_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.072)),
        material=graphite,
        name="intake_band_lower",
    )
    lower_housing.inertial = Inertial.from_geometry(
        Box((0.280, 0.280, 0.190)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    top_cap = model.part("top_cap")
    top_cap.visual(upper_shell_mesh, material=warm_white, name="upper_shell")
    top_cap.visual(
        Cylinder(radius=0.044, length=0.058),
        origin=Origin(xyz=(0.118, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_white,
        name="outlet_shroud",
    )
    top_cap.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.146, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="outlet_grille",
    )
    top_cap.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.134)),
        material=graphite,
        name="top_pivot_cap",
    )
    top_cap.inertial = Inertial.from_geometry(
        Box((0.300, 0.180, 0.160)),
        mass=0.75,
        origin=Origin(xyz=(0.030, 0.0, 0.070)),
    )

    filter_drawer = model.part("filter_drawer")
    filter_drawer.visual(
        Box((0.126, 0.124, 0.045)),
        origin=Origin(xyz=(0.0, -0.062, 0.0225)),
        material=filter_gray,
        name="cartridge_body",
    )
    filter_drawer.visual(
        Box((0.138, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, 0.001, 0.026)),
        material=charcoal,
        name="front_panel",
    )
    filter_drawer.visual(
        Box((0.064, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, 0.031)),
        material=soft_black,
        name="pull_handle",
    )
    filter_drawer.inertial = Inertial.from_geometry(
        Box((0.138, 0.130, 0.055)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.058, 0.027)),
    )

    model.articulation(
        "base_to_lower_housing",
        ArticulationType.FIXED,
        parent=base,
        child=lower_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.271)),
    )
    model.articulation(
        "housing_cap_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_housing,
        child=top_cap,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0),
    )
    model.articulation(
        "base_to_filter_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=filter_drawer,
        origin=Origin(xyz=(0.0, 0.147, 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.085,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_housing = object_model.get_part("lower_housing")
    top_cap = object_model.get_part("top_cap")
    filter_drawer = object_model.get_part("filter_drawer")
    housing_cap_spin = object_model.get_articulation("housing_cap_spin")
    drawer_slide = object_model.get_articulation("base_to_filter_drawer")

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

    ctx.check(
        "cap uses a continuous vertical spin joint",
        housing_cap_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(housing_cap_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={housing_cap_spin.articulation_type}, axis={housing_cap_spin.axis}",
    )
    ctx.check(
        "drawer uses a forward prismatic slide",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(drawer_slide.axis) == (0.0, 1.0, 0.0),
        details=f"type={drawer_slide.articulation_type}, axis={drawer_slide.axis}",
    )
    ctx.expect_contact(
        lower_housing,
        base,
        elem_a="support_neck",
        elem_b="neck_pad",
        name="globe housing is seated on the disc base",
    )
    ctx.expect_contact(
        top_cap,
        lower_housing,
        elem_a="upper_shell",
        elem_b="lower_shell",
        contact_tol=0.0005,
        name="top cap is supported on the equator seam",
    )
    with ctx.pose({housing_cap_spin: 0.0}):
        ctx.expect_overlap(
            top_cap,
            lower_housing,
            axes="xy",
            elem_a="upper_shell",
            elem_b="lower_shell",
            min_overlap=0.24,
            name="cap stays centered on the globe",
        )
        ctx.expect_within(
            filter_drawer,
            base,
            axes="x",
            inner_elem="cartridge_body",
            outer_elem="top_plate",
            margin=0.012,
            name="drawer body stays laterally within the base",
        )
        ctx.expect_overlap(
            filter_drawer,
            base,
            axes="y",
            elem_a="cartridge_body",
            elem_b="top_plate",
            min_overlap=0.110,
            name="closed drawer remains housed beneath the base roof",
        )
        ctx.expect_within(
            filter_drawer,
            base,
            axes="x",
            inner_elem="front_panel",
            outer_elem="slot_header",
            margin=0.001,
            name="drawer front stays centered in the opening",
        )

    outlet_at_rest = _aabb_center(ctx.part_element_world_aabb(top_cap, elem="outlet_shroud"))
    drawer_at_rest = ctx.part_world_position(filter_drawer)

    with ctx.pose({housing_cap_spin: math.pi / 2.0}):
        outlet_at_quarter_turn = _aabb_center(ctx.part_element_world_aabb(top_cap, elem="outlet_shroud"))

    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        drawer_extended = ctx.part_world_position(filter_drawer)
        ctx.expect_overlap(
            filter_drawer,
            base,
            axes="y",
            elem_a="cartridge_body",
            elem_b="top_plate",
            min_overlap=0.035,
            name="extended drawer still retains insertion in the base",
        )

    ctx.check(
        "top cap rotation redirects the outlet around the globe",
        outlet_at_rest is not None
        and outlet_at_quarter_turn is not None
        and outlet_at_rest[0] > 0.10
        and abs(outlet_at_rest[1]) < 0.02
        and outlet_at_quarter_turn[1] > 0.10
        and abs(outlet_at_quarter_turn[0]) < 0.03,
        details=f"rest={outlet_at_rest}, quarter_turn={outlet_at_quarter_turn}",
    )
    ctx.check(
        "filter drawer extends forward for replacement",
        drawer_at_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] > drawer_at_rest[1] + 0.06,
        details=f"rest={drawer_at_rest}, extended={drawer_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
