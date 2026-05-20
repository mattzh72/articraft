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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    segments: int = 56,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 64,
):
    ring = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        height=thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh(ring, name)


def _front_panel_mesh(
    name: str,
    *,
    outer_size: tuple[float, float],
    outer_radius: float,
    thickness: float,
    opening_size: tuple[float, float] | None = None,
    opening_radius: float = 0.0,
    opening_circle_radius: float | None = None,
    opening_center: tuple[float, float] = (0.0, 0.0),
):
    outer_profile = rounded_rect_profile(
        outer_size[0],
        outer_size[1],
        outer_radius,
        corner_segments=8,
    )
    holes: list[list[tuple[float, float]]] = []
    if opening_circle_radius is not None:
        holes.append(
            _circle_profile(
                opening_circle_radius,
                segments=64,
                center=opening_center,
            )
        )
    elif opening_size is not None:
        holes.append(
            _offset_profile(
                rounded_rect_profile(
                    opening_size[0],
                    opening_size[1],
                    opening_radius,
                    corner_segments=6,
                ),
                dx=opening_center[0],
                dy=opening_center[1],
            )
        )
    panel = ExtrudeWithHolesGeometry(
        outer_profile,
        holes,
        height=thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh(panel, name)


def _axis_close(
    axis: tuple[float, float, float] | None,
    target: tuple[float, float, float],
    tol: float = 1e-6,
) -> bool:
    if axis is None:
        return False
    return all(abs(a - b) <= tol for a, b in zip(axis, target))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_front_load_washer")

    appliance_white = model.material("appliance_white", rgba=(0.95, 0.96, 0.97, 1.0))
    pedestal_white = model.material("pedestal_white", rgba=(0.91, 0.92, 0.94, 1.0))
    warm_white = model.material("warm_white", rgba=(0.97, 0.97, 0.96, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.12, 0.14, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    chrome = model.material("chrome", rgba=(0.74, 0.77, 0.80, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.60, 0.72, 0.78, 0.28))
    drum_shadow = model.material("drum_shadow", rgba=(0.07, 0.08, 0.09, 1.0))

    pedestal_front = _front_panel_mesh(
        "pedestal_front_frame",
        outer_size=(0.620, 0.360),
        outer_radius=0.012,
        thickness=0.020,
        opening_size=(0.560, 0.250),
        opening_radius=0.008,
        opening_center=(0.0, -0.005),
    )
    washer_front = _front_panel_mesh(
        "washer_front_panel",
        outer_size=(0.580, 0.820),
        outer_radius=0.026,
        thickness=0.020,
        opening_circle_radius=0.215,
        opening_center=(0.0, -0.016),
    )
    door_frame_mesh = _ring_mesh(
        "washer_door_frame",
        outer_radius=0.240,
        inner_radius=0.177,
        thickness=0.060,
    )
    chrome_bezel_mesh = _ring_mesh(
        "washer_door_chrome_bezel",
        outer_radius=0.240,
        inner_radius=0.188,
        thickness=0.010,
    )
    door_inner_trim_mesh = _ring_mesh(
        "washer_door_inner_trim",
        outer_radius=0.186,
        inner_radius=0.165,
        thickness=0.012,
    )
    gasket_ring_mesh = _ring_mesh(
        "washer_gasket_ring",
        outer_radius=0.225,
        inner_radius=0.195,
        thickness=0.030,
    )

    pedestal = model.part("pedestal_body")
    pedestal.visual(
        Box((0.018, 0.700, 0.360)),
        origin=Origin(xyz=(-0.301, 0.0, 0.180)),
        material=pedestal_white,
        name="left_side",
    )
    pedestal.visual(
        Box((0.018, 0.700, 0.360)),
        origin=Origin(xyz=(0.301, 0.0, 0.180)),
        material=pedestal_white,
        name="right_side",
    )
    pedestal.visual(
        Box((0.584, 0.676, 0.014)),
        origin=Origin(xyz=(0.0, -0.004, 0.007)),
        material=pedestal_white,
        name="bottom_pan",
    )
    pedestal.visual(
        Box((0.620, 0.700, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.353)),
        material=warm_white,
        name="top_cap",
    )
    pedestal.visual(
        Box((0.584, 0.018, 0.324)),
        origin=Origin(xyz=(0.0, -0.341, 0.176)),
        material=pedestal_white,
        name="rear_panel",
    )
    pedestal.visual(
        Box((0.564, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.306, 0.330)),
        material=pedestal_white,
        name="front_frame",
    )
    pedestal.visual(
        Box((0.564, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.306, 0.025)),
        material=pedestal_white,
        name="front_sill",
    )
    pedestal.visual(
        Box((0.028, 0.012, 0.360)),
        origin=Origin(xyz=(-0.296, 0.306, 0.180)),
        material=pedestal_white,
        name="left_front_stile",
    )
    pedestal.visual(
        Box((0.028, 0.012, 0.360)),
        origin=Origin(xyz=(0.296, 0.306, 0.180)),
        material=pedestal_white,
        name="right_front_stile",
    )
    pedestal.visual(
        Box((0.012, 0.520, 0.024)),
        origin=Origin(xyz=(-0.286, -0.010, 0.175)),
        material=steel,
        name="left_outer_rail",
    )
    pedestal.visual(
        Box((0.012, 0.520, 0.024)),
        origin=Origin(xyz=(0.286, -0.010, 0.175)),
        material=steel,
        name="right_outer_rail",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.620, 0.700, 0.360)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )

    drawer = model.part("pedestal_drawer")
    drawer.visual(
        Box((0.540, 0.500, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=warm_white,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.012, 0.500, 0.158)),
        origin=Origin(xyz=(-0.264, 0.0, 0.091)),
        material=warm_white,
        name="left_wall",
    )
    drawer.visual(
        Box((0.012, 0.500, 0.158)),
        origin=Origin(xyz=(0.264, 0.0, 0.091)),
        material=warm_white,
        name="right_wall",
    )
    drawer.visual(
        Box((0.516, 0.012, 0.158)),
        origin=Origin(xyz=(0.0, -0.244, 0.091)),
        material=warm_white,
        name="back_wall",
    )
    drawer.visual(
        Box((0.516, 0.012, 0.158)),
        origin=Origin(xyz=(0.0, 0.244, 0.091)),
        material=warm_white,
        name="front_inner_wall",
    )
    drawer.visual(
        Box((0.590, 0.026, 0.245)),
        origin=Origin(xyz=(0.0, 0.262, 0.150)),
        material=pedestal_white,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.360, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.276, 0.192)),
        material=chrome,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.010, 0.500, 0.024)),
        origin=Origin(xyz=(-0.275, 0.0, 0.175)),
        material=steel,
        name="left_inner_rail",
    )
    drawer.visual(
        Box((0.010, 0.500, 0.024)),
        origin=Origin(xyz=(0.275, 0.0, 0.175)),
        material=steel,
        name="right_inner_rail",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.590, 0.520, 0.245)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.030, 0.123)),
    )

    washer = model.part("washer_body")
    washer.visual(
        Box((0.018, 0.660, 0.820)),
        origin=Origin(xyz=(-0.291, 0.0, 0.410)),
        material=appliance_white,
        name="left_side",
    )
    washer.visual(
        Box((0.018, 0.660, 0.820)),
        origin=Origin(xyz=(0.291, 0.0, 0.410)),
        material=appliance_white,
        name="right_side",
    )
    washer.visual(
        Box((0.564, 0.015, 0.804)),
        origin=Origin(xyz=(0.0, -0.3225, 0.402)),
        material=appliance_white,
        name="rear_panel",
    )
    washer.visual(
        Box((0.600, 0.660, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.830)),
        material=appliance_white,
        name="top_panel",
    )
    washer.visual(
        Box((0.564, 0.600, 0.024)),
        origin=Origin(xyz=(0.0, -0.020, 0.024)),
        material=appliance_white,
        name="bottom_pan",
    )
    washer.visual(
        washer_front,
        origin=Origin(xyz=(0.0, 0.292, 0.420)),
        material=appliance_white,
        name="front_panel",
    )
    washer.visual(
        Box((0.014, 0.042, 0.340)),
        origin=Origin(xyz=(-0.279, 0.323, 0.404)),
        material=appliance_white,
        name="hinge_bracket",
    )
    washer.visual(
        Cylinder(radius=0.014, length=0.096),
        origin=Origin(xyz=(-0.258, 0.344, 0.292)),
        material=chrome,
        name="hinge_knuckle_lower",
    )
    washer.visual(
        Cylinder(radius=0.014, length=0.096),
        origin=Origin(xyz=(-0.258, 0.344, 0.516)),
        material=chrome,
        name="hinge_knuckle_upper",
    )
    washer.visual(
        gasket_ring_mesh,
        origin=Origin(xyz=(0.0, 0.270, 0.404)),
        material=dark_trim,
        name="gasket_ring",
    )
    washer.visual(
        Cylinder(radius=0.195, length=0.210),
        origin=Origin(xyz=(0.0, 0.165, 0.404), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_shadow,
        name="drum_shadow",
    )
    washer.visual(
        Box((0.500, 0.012, 0.082)),
        origin=Origin(xyz=(0.0, 0.305, 0.758)),
        material=control_black,
        name="control_panel",
    )
    washer.visual(
        Box((0.160, 0.020, 0.038)),
        origin=Origin(xyz=(-0.150, 0.301, 0.756)),
        material=appliance_white,
        name="detergent_drawer",
    )
    washer.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.184, 0.309, 0.756), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="selector_knob",
    )
    for foot_index, (sx, sy) in enumerate(
        [(-0.205, -0.220), (-0.205, 0.220), (0.205, -0.220), (0.205, 0.220)],
        start=1,
    ):
        washer.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(xyz=(sx, sy, 0.006)),
            material=dark_trim,
            name=f"foot_{foot_index}",
        )
    washer.inertial = Inertial.from_geometry(
        Box((0.600, 0.660, 0.840)),
        mass=66.0,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
    )

    door = model.part("washer_door")
    door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(0.258, 0.0, 0.0)),
        material=pedestal_white,
        name="door_frame",
    )
    door.visual(
        chrome_bezel_mesh,
        origin=Origin(xyz=(0.258, 0.028, 0.0)),
        material=chrome,
        name="chrome_bezel",
    )
    door.visual(
        door_inner_trim_mesh,
        origin=Origin(xyz=(0.258, -0.014, 0.0)),
        material=dark_trim,
        name="inner_trim",
    )
    door.visual(
        Cylinder(radius=0.172, length=0.012),
        origin=Origin(xyz=(0.258, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoked_glass,
        name="glass",
    )
    door.visual(
        Box((0.050, 0.020, 0.340)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=pedestal_white,
        name="hinge_strap",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="barrel_2",
    )
    door.visual(
        Box((0.060, 0.022, 0.050)),
        origin=Origin(xyz=(0.423, 0.024, 0.0)),
        material=chrome,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.500, 0.080, 0.500)),
        mass=6.0,
        origin=Origin(xyz=(0.258, 0.0, 0.0)),
    )

    model.articulation(
        "washer_mount",
        ArticulationType.FIXED,
        parent=pedestal,
        child=washer,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.075, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.60,
            lower=0.0,
            upper=0.380,
        ),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=washer,
        child=door,
        origin=Origin(xyz=(-0.258, 0.344, 0.404)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.80,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal_body")
    drawer = object_model.get_part("pedestal_drawer")
    washer = object_model.get_part("washer_body")
    door = object_model.get_part("washer_door")
    drawer_slide = object_model.get_articulation("drawer_slide")
    door_hinge = object_model.get_articulation("door_hinge")

    pedestal.get_visual("front_frame")
    drawer.get_visual("drawer_front")
    washer.get_visual("front_panel")
    door.get_visual("door_frame")
    door.get_visual("chrome_bezel")

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

    door_limits = door_hinge.motion_limits
    drawer_limits = drawer_slide.motion_limits
    ctx.check(
        "door hinge axis is vertical",
        _axis_close(door_hinge.axis, (0.0, 0.0, 1.0)),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "door hinge opens with realistic range",
        door_limits is not None
        and door_limits.lower is not None
        and door_limits.upper is not None
        and abs(door_limits.lower) <= 1e-6
        and 1.20 <= door_limits.upper <= 1.70,
        details=f"limits={door_limits}",
    )
    ctx.check(
        "drawer slides forward on prismatic rails",
        _axis_close(drawer_slide.axis, (0.0, 1.0, 0.0)),
        details=f"axis={drawer_slide.axis}",
    )
    ctx.check(
        "drawer has full extension travel",
        drawer_limits is not None
        and drawer_limits.lower is not None
        and drawer_limits.upper is not None
        and abs(drawer_limits.lower) <= 1e-6
        and 0.36 <= drawer_limits.upper <= 0.42,
        details=f"limits={drawer_limits}",
    )

    with ctx.pose({door_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_contact(washer, pedestal, name="washer is seated on pedestal")
        ctx.expect_contact(drawer, pedestal, name="drawer contacts guide rails")
        ctx.expect_contact(door, washer, name="door remains mounted on hinge barrels")
        ctx.expect_gap(
            door,
            washer,
            axis="y",
            positive_elem="door_frame",
            negative_elem="front_panel",
            min_gap=0.008,
            max_gap=0.020,
            name="door frame sits slightly proud of front cabinet",
        )
        ctx.expect_overlap(
            door,
            washer,
            axes="xz",
            elem_a="door_frame",
            elem_b="front_panel",
            min_overlap=0.430,
            name="door covers the washer opening",
        )
        ctx.expect_origin_gap(
            drawer,
            pedestal,
            axis="y",
            min_gap=0.074,
            max_gap=0.076,
            name="drawer is closed flush in the pedestal",
        )

    with ctx.pose({drawer_slide: 0.380}):
        ctx.expect_contact(drawer, pedestal, name="drawer stays supported at full extension")
        ctx.expect_origin_gap(
            drawer,
            pedestal,
            axis="y",
            min_gap=0.454,
            max_gap=0.456,
            name="drawer reaches full extension travel",
        )

    with ctx.pose({door_hinge: 1.25, drawer_slide: 0.380}):
        ctx.expect_contact(door, washer, name="door remains supported while open")
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in the primary open pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
