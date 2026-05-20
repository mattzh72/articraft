from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

BARREL_OUTER_RADIUS = 0.00645
BARREL_INNER_RADIUS = 0.00535
BARREL_REAR_Z = 0.084
PLUNGER_TRAVEL = 0.038


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_barrel_shell():
    outer_wall = [
        (0.0010, 0.0002),
        (0.00115, 0.0042),
        (0.0018, 0.0062),
        (0.00195, 0.0092),
        (0.0039, 0.0116),
        (0.00435, 0.0155),
        (BARREL_OUTER_RADIUS, 0.0180),
        (BARREL_OUTER_RADIUS, 0.0790),
        (0.00695, 0.0812),
        (0.00695, BARREL_REAR_Z),
    ]
    inner_wall = [
        (0.00045, 0.0100),
        (0.00130, 0.0128),
        (0.00172, 0.0178),
        (BARREL_INNER_RADIUS, 0.0180),
        (BARREL_INNER_RADIUS, 0.0860),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_wall,
        inner_wall,
        segments=144,
        start_cap="flat",
        end_cap="flat",
    )


def _build_rear_collar():
    outer_wall = [
        (0.00635, -0.00155),
        (0.00695, -0.00115),
        (0.00720, -0.00010),
        (0.00720, 0.00080),
        (0.00685, 0.00135),
        (0.00635, 0.00155),
    ]
    inner_wall = [
        (0.00205, -0.00195),
        (0.00205, 0.00195),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_wall,
        inner_wall,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_nozzle_collar():
    outer_wall = [
        (0.00415, -0.00080),
        (0.00445, -0.00025),
        (0.00445, 0.00045),
        (0.00415, 0.00080),
    ]
    inner_wall = [
        (0.00210, -0.00110),
        (0.00210, 0.00110),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_wall,
        inner_wall,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_guide_sleeve():
    profile = [
        (0.0, -0.0070),
        (0.00210, -0.0070),
        (0.00265, -0.0061),
        (0.00295, -0.0042),
        (0.00295, 0.0048),
        (0.00255, 0.0064),
        (0.00215, 0.0070),
        (0.0, 0.0070),
    ]
    return LatheGeometry(profile, segments=72)


def _build_thumb_hub():
    profile = [
        (0.0, -0.0042),
        (0.00200, -0.0042),
        (0.00340, -0.0032),
        (0.00420, -0.0012),
        (0.00465, 0.0014),
        (0.00500, 0.0040),
        (0.0, 0.0040),
    ]
    return LatheGeometry(profile, segments=72)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_syringe", assets=ASSETS)

    clear_polymer = model.material("clear_polymer", rgba=(0.86, 0.91, 0.95, 0.34))
    pearl_polymer = model.material("pearl_polymer", rgba=(0.93, 0.94, 0.95, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.35, 0.38, 0.42, 1.0))
    dark_elastomer = model.material("dark_elastomer", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.69, 0.72, 0.75, 1.0))
    print_ink = model.material("print_ink", rgba=(0.22, 0.26, 0.30, 1.0))

    finger_flange_mesh = _mesh(
        "finger_flange.obj",
        ExtrudeGeometry(
            rounded_rect_profile(0.0115, 0.0064, radius=0.0022, corner_segments=8),
            0.0024,
        ),
    )
    thumb_plate_mesh = _mesh(
        "thumb_plate.obj",
        ExtrudeGeometry(
            rounded_rect_profile(0.022, 0.0084, radius=0.0026, corner_segments=8),
            0.0040,
        ),
    )
    thumb_inset_mesh = _mesh(
        "thumb_inset.obj",
        ExtrudeGeometry(
            rounded_rect_profile(0.015, 0.0056, radius=0.0019, corner_segments=8),
            0.0012,
        ),
    )

    barrel = model.part("barrel")
    barrel.visual(
        _mesh("barrel_shell.obj", _build_barrel_shell()),
        material=clear_polymer,
        name="barrel_shell",
    )
    barrel.visual(
        _mesh("rear_collar.obj", _build_rear_collar()),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=pearl_polymer,
        name="rear_collar",
    )
    barrel.visual(
        _mesh("nozzle_collar.obj", _build_nozzle_collar()),
        origin=Origin(xyz=(0.0, 0.0, 0.0130)),
        material=satin_metal,
        name="nozzle_collar",
    )
    barrel.visual(
        finger_flange_mesh,
        origin=Origin(xyz=(0.0109, 0.0, 0.0718)),
        material=pearl_polymer,
        name="finger_flange_right",
    )
    barrel.visual(
        finger_flange_mesh,
        origin=Origin(xyz=(-0.0109, 0.0, 0.0718)),
        material=pearl_polymer,
        name="finger_flange_left",
    )
    for index in range(12):
        z_pos = 0.026 + (0.004 * index)
        major = index in (0, 5, 10)
        width = 0.0048 if major else 0.0032
        barrel.visual(
            Box((0.00030, width, 0.00052)),
            origin=Origin(xyz=(0.00610, 0.0, z_pos)),
            material=print_ink,
            name=f"graduation_{index:02d}",
        )
    barrel.inertial = Inertial.from_geometry(
        Box((0.028, 0.016, BARREL_REAR_Z)),
        mass=0.032,
        origin=Origin(xyz=(0.0, 0.0, BARREL_REAR_Z * 0.5)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.00165, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_graphite,
        name="plunger_rod",
    )
    plunger.visual(
        _mesh("guide_sleeve.obj", _build_guide_sleeve()),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=satin_graphite,
        name="guide_sleeve",
    )
    plunger.visual(
        Cylinder(radius=0.00500, length=0.0064),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=dark_elastomer,
        name="stopper_body",
    )
    plunger.visual(
        Cylinder(radius=0.00535, length=0.0011),
        origin=Origin(xyz=(0.0, 0.0, -0.0263)),
        material=dark_elastomer,
        name="stopper_seal_front",
    )
    plunger.visual(
        Cylinder(radius=0.00535, length=0.0011),
        origin=Origin(xyz=(0.0, 0.0, -0.0217)),
        material=dark_elastomer,
        name="stopper_seal_rear",
    )
    plunger.visual(
        _mesh("thumb_hub.obj", _build_thumb_hub()),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=pearl_polymer,
        name="thumb_hub",
    )
    plunger.visual(
        thumb_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=pearl_polymer,
        name="thumb_plate",
    )
    plunger.visual(
        thumb_inset_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0464)),
        material=satin_graphite,
        name="thumb_inset",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.022, 0.010, 0.074)),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, BARREL_REAR_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.12,
            lower=0.0,
            upper=PLUNGER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    plunger_slide = object_model.get_articulation("plunger_slide")
    barrel_shell = barrel.get_visual("barrel_shell")
    rear_collar = barrel.get_visual("rear_collar")
    nozzle_collar = barrel.get_visual("nozzle_collar")
    stopper_body = plunger.get_visual("stopper_body")
    seal_front = plunger.get_visual("stopper_seal_front")
    seal_rear = plunger.get_visual("stopper_seal_rear")
    plunger_rod = plunger.get_visual("plunger_rod")
    thumb_plate = plunger.get_visual("thumb_plate")

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
    ctx.allow_overlap(
        plunger,
        barrel,
        elem_a=seal_front,
        elem_b=barrel_shell,
        reason="The front elastomer seal is intentionally preloaded into the barrel bore for realistic syringe sealing.",
    )
    ctx.allow_overlap(
        plunger,
        barrel,
        elem_a=seal_rear,
        elem_b=barrel_shell,
        reason="The rear elastomer seal is intentionally preloaded into the barrel bore for realistic guided plunger friction.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "plunger_axis_is_barrel_axial",
        tuple(round(v, 6) for v in plunger_slide.axis) == (0.0, 0.0, -1.0),
        f"Expected axial prismatic axis, got {plunger_slide.axis!r}",
    )
    ctx.check(
        "plunger_limits_are_believable",
        plunger_slide.motion_limits is not None
        and abs(plunger_slide.motion_limits.lower - 0.0) < 1e-9
        and abs(plunger_slide.motion_limits.upper - PLUNGER_TRAVEL) < 1e-9,
        f"Unexpected limits: {plunger_slide.motion_limits!r}",
    )
    ctx.expect_origin_distance(plunger, barrel, axes="xy", max_dist=0.0002)
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="xy",
        elem_a=stopper_body,
        elem_b=barrel_shell,
        min_overlap=0.0098,
    )
    ctx.expect_within(
        plunger,
        barrel,
        axes="xy",
        inner_elem=stopper_body,
        outer_elem=barrel_shell,
        margin=0.0007,
    )
    ctx.expect_within(
        plunger,
        barrel,
        axes="xy",
        inner_elem=plunger_rod,
        outer_elem=rear_collar,
        margin=0.0010,
    )
    ctx.expect_contact(
        plunger,
        barrel,
        elem_a=seal_front,
        elem_b=barrel_shell,
    )
    ctx.expect_contact(
        plunger,
        barrel,
        elem_a=seal_rear,
        elem_b=barrel_shell,
    )
    ctx.expect_gap(
        plunger,
        barrel,
        axis="z",
        positive_elem=thumb_plate,
        negative_elem=rear_collar,
        min_gap=0.040,
        max_gap=0.050,
    )
    ctx.expect_gap(
        plunger,
        barrel,
        axis="z",
        positive_elem=stopper_body,
        negative_elem=nozzle_collar,
        min_gap=0.035,
    )

    rest_pos = ctx.part_world_position(plunger)
    limits = plunger_slide.motion_limits
    assert rest_pos is not None
    assert limits is not None
    with ctx.pose({plunger_slide: limits.upper}):
        pushed_pos = ctx.part_world_position(plunger)
        assert pushed_pos is not None
        ctx.check(
            "plunger_translates_along_axis_only",
            abs((rest_pos[2] - pushed_pos[2]) - limits.upper) < 1e-4
            and abs(rest_pos[0] - pushed_pos[0]) < 1e-6
            and abs(rest_pos[1] - pushed_pos[1]) < 1e-6,
            f"Rest position {rest_pos!r}, pushed position {pushed_pos!r}, expected z travel {limits.upper:.4f}",
        )
        ctx.expect_origin_distance(plunger, barrel, axes="xy", max_dist=0.0002)
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="xy",
            elem_a=stopper_body,
            elem_b=barrel_shell,
            min_overlap=0.0098,
        )
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            inner_elem=stopper_body,
            outer_elem=barrel_shell,
            margin=0.0007,
        )
        ctx.expect_gap(
            plunger,
            barrel,
            axis="z",
            positive_elem=thumb_plate,
            negative_elem=rear_collar,
            min_gap=0.004,
        )
        ctx.expect_gap(
            plunger,
            barrel,
            axis="z",
            positive_elem=stopper_body,
            negative_elem=nozzle_collar,
            min_gap=0.003,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
