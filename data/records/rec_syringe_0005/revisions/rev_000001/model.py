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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_barrel_shell():
    outer_profile = [
        (0.0096, -0.0040),
        (0.0092, -0.0010),
        (0.00845, 0.0035),
        (0.00825, 0.0100),
        (0.00825, 0.0715),
        (0.00805, 0.0760),
        (0.00710, 0.0810),
        (0.00510, 0.0880),
        (0.00385, 0.0930),
        (0.00325, 0.0975),
        (0.00225, 0.1015),
        (0.00180, 0.1050),
    ]
    inner_profile = [
        (0.00725, 0.0000),
        (0.00725, 0.0705),
        (0.00695, 0.0745),
        (0.00590, 0.0800),
        (0.00370, 0.0890),
        (0.00180, 0.0990),
        (0.00080, 0.1050),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=80,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )
    shell.rotate_y(math.pi / 2.0)
    return shell


def _build_paddle(span_y: float, span_z: float, thickness: float, corner: float):
    paddle = ExtrudeGeometry(
        rounded_rect_profile(span_z, span_y, corner, corner_segments=8),
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    paddle.rotate_y(math.pi / 2.0)
    return paddle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_syringe", assets=ASSETS)

    clear_polymer = model.material("clear_polymer", rgba=(0.88, 0.94, 0.98, 0.26))
    satin_polymer = model.material("satin_polymer", rgba=(0.95, 0.96, 0.97, 1.0))
    matte_polymer = model.material("matte_polymer", rgba=(0.87, 0.88, 0.90, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.16, 0.17, 0.18, 1.0))
    print_ink = model.material("print_ink", rgba=(0.20, 0.33, 0.43, 1.0))
    satin_hardware = model.material("satin_hardware", rgba=(0.73, 0.76, 0.79, 1.0))

    barrel_shell_mesh = _save_mesh("syringe_barrel_shell.obj", _build_barrel_shell())
    finger_flange_mesh = _save_mesh(
        "syringe_finger_flange.obj",
        _build_paddle(span_y=0.026, span_z=0.011, thickness=0.0028, corner=0.0034),
    )
    thumb_pad_mesh = _save_mesh(
        "syringe_thumb_pad.obj",
        _build_paddle(span_y=0.032, span_z=0.018, thickness=0.0032, corner=0.0048),
    )
    thumb_pad_cap_mesh = _save_mesh(
        "syringe_thumb_pad_cap.obj",
        _build_paddle(span_y=0.024, span_z=0.012, thickness=0.0012, corner=0.0032),
    )

    barrel = model.part("barrel")
    barrel.visual(barrel_shell_mesh, material=clear_polymer, name="barrel_shell")
    barrel.visual(
        finger_flange_mesh,
        origin=Origin(xyz=(-0.0024, 0.0145, -0.0015)),
        material=satin_polymer,
        name="finger_flange_right",
    )
    barrel.visual(
        finger_flange_mesh,
        origin=Origin(xyz=(-0.0024, -0.0145, -0.0015)),
        material=satin_polymer,
        name="finger_flange_left",
    )
    barrel.visual(
        Cylinder(radius=0.0100, length=0.0038),
        origin=Origin(xyz=(-0.0008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_polymer,
        name="rear_collar",
    )
    barrel.visual(
        Box((0.0048, 0.0062, 0.0046)),
        origin=Origin(xyz=(-0.0010, 0.0116, 0.0)),
        material=satin_polymer,
        name="flange_gusset_right",
    )
    barrel.visual(
        Box((0.0048, 0.0062, 0.0046)),
        origin=Origin(xyz=(-0.0010, -0.0116, 0.0)),
        material=satin_polymer,
        name="flange_gusset_left",
    )
    barrel.visual(
        Cylinder(radius=0.0052, length=0.0052),
        origin=Origin(xyz=(0.0865, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_hardware,
        name="tip_band",
    )
    barrel.visual(
        Box((0.0115, 0.0038, 0.00016)),
        origin=Origin(xyz=(0.030, -0.0024, 0.00818)),
        material=print_ink,
        name="volume_label",
    )

    mark_start = 0.014
    mark_pitch = 0.0054
    mark_z = 0.00818
    for index in range(11):
        mark_x = mark_start + index * mark_pitch
        barrel.visual(
            Box((0.00050, 0.0058, 0.00016)),
            origin=Origin(xyz=(mark_x, 0.0004, mark_z)),
            material=print_ink,
            name=f"major_mark_{index:02d}",
        )
        if index < 10:
            barrel.visual(
                Box((0.00038, 0.0036, 0.00016)),
                origin=Origin(xyz=(mark_x + (mark_pitch * 0.5), 0.0014, mark_z)),
                material=print_ink,
                name=f"minor_mark_{index:02d}",
            )

    barrel.inertial = Inertial.from_geometry(
        Box((0.112, 0.055, 0.020)),
        mass=0.018,
        origin=Origin(xyz=(0.051, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Box((0.072, 0.0048, 0.0018)),
        origin=Origin(xyz=(-0.022, 0.0, 0.0)),
        material=satin_polymer,
        name="rod_web_wide",
    )
    plunger.visual(
        Box((0.072, 0.0018, 0.0048)),
        origin=Origin(xyz=(-0.022, 0.0, 0.0)),
        material=satin_polymer,
        name="rod_web_tall",
    )
    plunger.visual(
        Cylinder(radius=0.0041, length=0.0110),
        origin=Origin(xyz=(-0.0593, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_polymer,
        name="thumb_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0064, length=0.0046),
        origin=Origin(xyz=(-0.0520, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_polymer,
        name="thumb_transition",
    )
    plunger.visual(
        thumb_pad_mesh,
        origin=Origin(xyz=(-0.0660, 0.0, 0.0)),
        material=matte_polymer,
        name="thumb_pad",
    )
    plunger.visual(
        thumb_pad_cap_mesh,
        origin=Origin(xyz=(-0.0646, 0.0, 0.0)),
        material=satin_polymer,
        name="thumb_pad_cap",
    )
    plunger.visual(
        Cylinder(radius=0.0030, length=0.0090),
        origin=Origin(xyz=(0.0015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_polymer,
        name="stopper_post",
    )
    plunger.visual(
        Cylinder(radius=0.0066, length=0.0100),
        origin=Origin(xyz=(0.0115, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_rubber,
        name="stopper_core",
    )
    plunger.visual(
        Cylinder(radius=0.00725, length=0.0018),
        origin=Origin(xyz=(0.0074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_rubber,
        name="stopper_rear_seal",
    )
    plunger.visual(
        Cylinder(radius=0.00725, length=0.0018),
        origin=Origin(xyz=(0.0156, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_rubber,
        name="stopper_front_seal",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.091, 0.033, 0.018)),
        mass=0.010,
        origin=Origin(xyz=(-0.027, 0.0, 0.0)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.22,
            lower=0.0,
            upper=0.055,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    plunger_slide = object_model.get_articulation("plunger_slide")

    barrel_shell = barrel.get_visual("barrel_shell")
    tip_band = barrel.get_visual("tip_band")
    volume_label = barrel.get_visual("volume_label")
    major_mark_00 = barrel.get_visual("major_mark_00")
    major_mark_10 = barrel.get_visual("major_mark_10")

    thumb_pad = plunger.get_visual("thumb_pad")
    stopper_rear = plunger.get_visual("stopper_rear_seal")
    stopper_front = plunger.get_visual("stopper_front_seal")

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
    ctx.fail_if_isolated_parts(max_pose_samples=16, name="sampled_pose_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    barrel_aabb = ctx.part_world_aabb(barrel)
    assert barrel_aabb is not None
    barrel_length = barrel_aabb[1][0] - barrel_aabb[0][0]
    barrel_span_y = barrel_aabb[1][1] - barrel_aabb[0][1]
    barrel_span_z = barrel_aabb[1][2] - barrel_aabb[0][2]

    ctx.check(
        "barrel_length_realistic",
        0.105 <= barrel_length <= 0.120,
        f"Expected overall barrel length near a 10 mL clinical syringe, got {barrel_length:.4f} m.",
    )
    ctx.check(
        "barrel_cross_section_realistic",
        0.050 <= barrel_span_y <= 0.056 and 0.016 <= barrel_span_z <= 0.021,
        (
            "Expected a narrow cylindrical barrel with finger flanges and clinical-scale diameter, "
            f"got y-span {barrel_span_y:.4f} m and z-span {barrel_span_z:.4f} m."
        ),
    )

    limits = plunger_slide.motion_limits
    assert limits is not None and limits.lower is not None and limits.upper is not None
    ctx.check(
        "plunger_travel_limits_realistic",
        0.050 <= limits.upper - limits.lower <= 0.058 and abs(limits.lower) < 1e-9,
        (
            "Expected guided plunger travel around 50-58 mm from a fully retracted stop, "
            f"got lower={limits.lower:.4f}, upper={limits.upper:.4f}."
        ),
    )
    ctx.check(
        "graduation_features_present",
        len(barrel.visuals) >= 26 and volume_label.name == "volume_label",
        "Expected a labeled, graduated barrel with multiple major and minor marks.",
    )
    barrel_visual_names = {visual.name for visual in barrel.visuals}
    plunger_visual_names = {visual.name for visual in plunger.visuals}
    ctx.check(
        "premium_detail_features_present",
        {
            "rear_collar",
            "flange_gusset_right",
            "flange_gusset_left",
        }.issubset(barrel_visual_names)
        and {"thumb_transition", "thumb_pad_cap"}.issubset(plunger_visual_names),
        "Expected refined premium detailing at the barrel hub, flange roots, and thumb interface.",
    )

    ctx.expect_overlap(plunger, barrel, axes="yz", min_overlap=0.012, elem_a=stopper_front, elem_b=barrel_shell)
    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        margin=0.001,
        inner_elem=stopper_front,
        outer_elem=barrel_shell,
    )
    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        margin=0.001,
        inner_elem=stopper_rear,
        outer_elem=barrel_shell,
    )
    ctx.expect_contact(
        plunger,
        barrel,
        elem_a=stopper_front,
        elem_b=barrel_shell,
        contact_tol=0.00025,
        name="front_seal_contact_at_rest",
    )

    rest_pos = ctx.part_world_position(plunger)
    assert rest_pos is not None
    with ctx.pose({plunger_slide: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="plunger_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="plunger_lower_no_floating")
        ctx.expect_contact(
            plunger,
            barrel,
            elem_a=stopper_rear,
            elem_b=barrel_shell,
            contact_tol=0.00025,
            name="rear_seal_contact_lower",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="x",
            min_gap=0.060,
            negative_elem=thumb_pad,
            name="thumb_pad_retracted_clearance",
        )

    with ctx.pose({plunger_slide: limits.upper}):
        pushed_pos = ctx.part_world_position(plunger)
        assert pushed_pos is not None
        ctx.check(
            "plunger_moves_only_along_barrel_axis",
            pushed_pos[0] > rest_pos[0] + 0.053
            and abs(pushed_pos[1] - rest_pos[1]) < 1e-6
            and abs(pushed_pos[2] - rest_pos[2]) < 1e-6,
            (
                "Expected prismatic motion to stay coaxial with the barrel axis, "
                f"rest={rest_pos}, pushed={pushed_pos}."
            ),
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="plunger_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="plunger_upper_no_floating")
        ctx.expect_contact(
            plunger,
            barrel,
            elem_a=stopper_front,
            elem_b=barrel_shell,
            contact_tol=0.00025,
            name="front_seal_contact_upper",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="x",
            min_gap=0.008,
            positive_elem=tip_band,
            negative_elem=stopper_front,
            name="stopper_clears_nozzle_band",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="x",
            min_gap=0.001,
            negative_elem=thumb_pad,
            name="thumb_pad_stays_clear_of_barrel",
        )

    ctx.check(
        "graduation_span_balanced",
        major_mark_10.origin.xyz[0] > major_mark_00.origin.xyz[0] + 0.050,
        "Expected major graduations to cover most of the usable barrel length.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
