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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_HEIGHT = 0.278
BODY_RADIUS = 0.070
CAP_SEAT_Z = BODY_HEIGHT
FILTER_SEAT_Z = 0.266
FILTER_TRAVEL = 0.105


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_body_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.048, 0.000),
            (0.062, 0.006),
            (0.069, 0.024),
            (0.070, 0.182),
            (0.068, 0.248),
            (0.066, BODY_HEIGHT),
        ],
        [
            (0.000, 0.012),
            (0.055, 0.026),
            (0.054, 0.248),
            (0.056, BODY_HEIGHT),
        ],
        segments=72,
    )


def _build_cap_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.060, 0.000),
            (0.070, 0.004),
            (0.075, 0.013),
            (0.072, 0.024),
        ],
        [
            (0.056, 0.000),
            (0.057, 0.010),
            (0.056, 0.024),
        ],
        segments=64,
    )


def _build_filter_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.018, -0.154),
            (0.022, -0.150),
            (0.030, -0.146),
            (0.046, -0.140),
            (0.049, -0.132),
            (0.050, -0.024),
            (0.048, 0.002),
            (0.045, 0.006),
        ],
        [
            (0.000, -0.138),
            (0.020, -0.132),
            (0.022, -0.024),
            (0.021, 0.004),
        ],
        segments=64,
    )


def _build_filter_handle():
    return tube_from_spline_points(
        [
            (-0.014, 0.000, 0.004),
            (-0.016, 0.000, 0.015),
            (0.000, 0.000, 0.029),
            (0.016, 0.000, 0.015),
            (0.014, 0.000, 0.004),
        ],
        radius=0.0018,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.94, 0.95, 0.96, 1.0))
    cap_charcoal = model.material("cap_charcoal", rgba=(0.23, 0.25, 0.28, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.09, 0.10, 1.0))
    filter_cream = model.material("filter_cream", rgba=(0.95, 0.93, 0.84, 1.0))
    handle_bluegray = model.material("handle_bluegray", rgba=(0.56, 0.62, 0.68, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.70, 0.73, 0.76, 1.0))

    tower_body = model.part("tower_body")
    tower_body.visual(
        _mesh("purifier_tower_shell", _build_body_shell()),
        material=shell_white,
        name="tower_shell",
    )
    tower_body.visual(
        Cylinder(radius=0.055, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber_black,
        name="base_pad",
    )
    tower_body.visual(
        Cylinder(radius=0.024, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=accent_gray,
        name="filter_support_pedestal",
    )
    tower_body.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(
            xyz=(0.068, 0.0, 0.212),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=accent_gray,
        name="control_button",
    )
    tower_body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_RADIUS, length=BODY_HEIGHT),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    top_cap = model.part("top_cap")
    top_cap.visual(
        _mesh("purifier_cap_shell", _build_cap_shell()),
        material=cap_charcoal,
        name="cap_shell",
    )
    top_cap.visual(
        Box((0.022, 0.012, 0.005)),
        origin=Origin(xyz=(0.082, 0.0, 0.013)),
        material=cap_charcoal,
        name="grip_tab_right",
    )
    top_cap.visual(
        Box((0.022, 0.012, 0.005)),
        origin=Origin(xyz=(-0.082, 0.0, 0.013)),
        material=cap_charcoal,
        name="grip_tab_left",
    )
    top_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=0.024),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    filter_cartridge = model.part("filter_cartridge")
    filter_cartridge.visual(
        _mesh("purifier_filter_shell", _build_filter_shell()),
        material=filter_cream,
        name="filter_shell",
    )
    for index in range(12):
        angle = index * math.tau / 12.0
        filter_cartridge.visual(
            Box((0.006, 0.008, 0.122)),
            origin=Origin(
                xyz=(0.046 * math.cos(angle), 0.046 * math.sin(angle), -0.072),
                rpy=(0.0, 0.0, angle),
            ),
            material=filter_cream,
            name=f"pleat_rib_{index:02d}",
        )
    filter_cartridge.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=handle_bluegray,
        name="pull_hub",
    )
    filter_cartridge.visual(
        _mesh("purifier_filter_handle", _build_filter_handle()),
        material=handle_bluegray,
        name="pull_handle",
    )
    filter_cartridge.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.160),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    model.articulation(
        "cap_unlock",
        ArticulationType.REVOLUTE,
        parent=tower_body,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "filter_slide",
        ArticulationType.PRISMATIC,
        parent=tower_body,
        child=filter_cartridge,
        origin=Origin(xyz=(0.0, 0.0, FILTER_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.12,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower_body = object_model.get_part("tower_body")
    top_cap = object_model.get_part("top_cap")
    filter_cartridge = object_model.get_part("filter_cartridge")
    cap_unlock = object_model.get_articulation("cap_unlock")
    filter_slide = object_model.get_articulation("filter_slide")

    cap_limits = cap_unlock.motion_limits
    filter_limits = filter_slide.motion_limits

    ctx.check(
        "cap unlock joint rotates about vertical axis",
        cap_unlock.articulation_type == ArticulationType.REVOLUTE
        and cap_unlock.axis == (0.0, 0.0, 1.0)
        and cap_limits is not None
        and math.isclose(cap_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and cap_limits.upper is not None
        and 0.35 < cap_limits.upper < 0.60,
        details=f"axis={cap_unlock.axis}, limits={cap_limits}",
    )
    ctx.check(
        "filter cartridge slides upward from the top opening",
        filter_slide.articulation_type == ArticulationType.PRISMATIC
        and filter_slide.axis == (0.0, 0.0, 1.0)
        and filter_limits is not None
        and math.isclose(filter_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and filter_limits.upper is not None
        and 0.09 < filter_limits.upper < 0.13,
        details=f"axis={filter_slide.axis}, limits={filter_limits}",
    )

    ctx.expect_origin_gap(
        top_cap,
        tower_body,
        axis="z",
        min_gap=0.274,
        max_gap=0.282,
        name="cap sits on top of the purifier tower",
    )
    ctx.expect_gap(
        top_cap,
        tower_body,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="tower_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="cap collar seats on the tower lip",
    )
    ctx.expect_overlap(
        top_cap,
        tower_body,
        axes="xy",
        min_overlap=0.12,
        name="top cap stays centered on the tower",
    )
    ctx.expect_gap(
        filter_cartridge,
        tower_body,
        axis="z",
        positive_elem="filter_shell",
        negative_elem="filter_support_pedestal",
        max_gap=0.001,
        max_penetration=0.0,
        name="resting filter cartridge is supported from below",
    )
    ctx.expect_within(
        filter_cartridge,
        tower_body,
        axes="xy",
        margin=0.006,
        name="resting filter stays centered inside the tower",
    )

    rest_filter_pos = ctx.part_world_position(filter_cartridge)
    with ctx.pose({cap_unlock: cap_limits.upper, filter_slide: filter_limits.upper}):
        ctx.expect_within(
            filter_cartridge,
            tower_body,
            axes="xy",
            margin=0.006,
            name="extended filter stays aligned with the tower opening",
        )
        ctx.expect_overlap(
            filter_cartridge,
            tower_body,
            axes="z",
            min_overlap=0.045,
            name="extended filter retains insertion in the housing",
        )
        raised_filter_pos = ctx.part_world_position(filter_cartridge)

    ctx.check(
        "filter rises upward when pulled",
        rest_filter_pos is not None
        and raised_filter_pos is not None
        and raised_filter_pos[2] > rest_filter_pos[2] + 0.09,
        details=f"rest={rest_filter_pos}, raised={raised_filter_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
