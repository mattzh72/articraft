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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _build_bottle_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.036, 0.000),
            (0.042, 0.010),
            (0.042, 0.182),
            (0.040, 0.205),
            (0.034, 0.220),
            (0.026, 0.232),
            (0.020, 0.240),
            (0.020, 0.248),
        ],
        [
            (0.000, 0.003),
            (0.034, 0.009),
            (0.036, 0.180),
            (0.034, 0.203),
            (0.028, 0.218),
            (0.020, 0.230),
            (0.014, 0.238),
            (0.014, 0.246),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )


def _build_collar_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.031, 0.000),
            (0.031, 0.012),
            (0.029, 0.020),
            (0.026, 0.036),
        ],
        [
            (0.020, 0.000),
            (0.020, 0.020),
            (0.008, 0.020),
            (0.008, 0.034),
            (0.010, 0.036),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_nozzle_socket_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0130, -0.0060),
            (0.0130, 0.0040),
            (0.0116, 0.0120),
            (0.0100, 0.0150),
        ],
        [
            (0.0044, -0.0060),
            (0.0044, -0.0020),
            (0.0108, -0.0020),
            (0.0108, 0.0100),
            (0.0092, 0.0150),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def _build_nozzle_cap_mesh() -> MeshGeometry:
    cap = ExtrudeGeometry.centered(
        rounded_rect_profile(0.050, 0.030, 0.012, corner_segments=10),
        0.012,
        cap=True,
        closed=True,
    )
    cap.translate(0.0, 0.0, 0.018)

    top_pad = ExtrudeGeometry.centered(
        rounded_rect_profile(0.032, 0.020, 0.008, corner_segments=8),
        0.003,
        cap=True,
        closed=True,
    )
    top_pad.translate(-0.004, 0.0, 0.025)

    pedestal = CylinderGeometry(radius=0.0105, height=0.016, radial_segments=32).translate(
        0.0,
        0.0,
        0.008,
    )
    front_blend = CylinderGeometry(radius=0.0100, height=0.016, radial_segments=28)
    front_blend.rotate_y(math.pi / 2.0).translate(0.016, 0.0, 0.016)

    return _merge_geometries([pedestal, cap, top_pad, front_blend])


def _build_spout_mesh() -> MeshGeometry:
    boss = CylinderGeometry(radius=0.0074, height=0.018, radial_segments=24)
    boss.rotate_y(math.pi / 2.0).translate(0.021, 0.0, 0.016)

    spout = tube_from_spline_points(
        [
            (0.024, 0.000, 0.016),
            (0.040, 0.000, 0.017),
            (0.058, 0.000, 0.014),
            (0.078, 0.000, 0.006),
        ],
        radius=0.0042,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )

    tip = CylinderGeometry(radius=0.0035, height=0.010, radial_segments=20)
    tip.rotate_y(math.pi / 2.0).translate(0.082, 0.0, 0.006)

    return _merge_geometries([boss, spout, tip])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dish_soap_pump_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.75, 0.86, 0.72, 0.95))
    pump_plastic = model.material("pump_plastic", rgba=(0.95, 0.95, 0.93, 1.0))
    stem_plastic = model.material("stem_plastic", rgba=(0.98, 0.98, 0.98, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_build_bottle_shell_mesh(), "bottle_shell"),
        material=bottle_plastic,
        name="bottle_shell",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.043, length=0.248),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
    )

    collar = model.part("collar")
    collar.visual(
        mesh_from_geometry(_build_collar_mesh(), "collar_shell"),
        material=pump_plastic,
        name="collar_shell",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.036),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    stem = model.part("plunger_stem")
    stem.visual(
        Cylinder(radius=0.0072, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=stem_plastic,
        name="guide_sleeve",
    )
    stem.visual(
        Cylinder(radius=0.0145, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=stem_plastic,
        name="stop_flange",
    )
    stem.visual(
        Cylinder(radius=0.0055, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=stem_plastic,
        name="exposed_stem",
    )
    stem.visual(
        Cylinder(radius=0.0042, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=stem_plastic,
        name="retainer_neck",
    )
    stem.visual(
        Cylinder(radius=0.0102, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=stem_plastic,
        name="retainer_disc",
    )
    stem.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0145, length=0.044),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    nozzle = model.part("nozzle_head")
    nozzle.visual(
        mesh_from_geometry(_build_nozzle_socket_mesh(), "nozzle_socket"),
        material=pump_plastic,
        name="socket",
    )
    nozzle.visual(
        mesh_from_geometry(_build_nozzle_cap_mesh(), "nozzle_cap"),
        material=pump_plastic,
        name="cap",
    )
    nozzle.visual(
        mesh_from_geometry(_build_spout_mesh(), "nozzle_spout"),
        material=pump_plastic,
        name="spout",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.088, 0.032, 0.032)),
        mass=0.08,
        origin=Origin(xyz=(0.032, 0.0, 0.014)),
    )

    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.248)),
    )
    model.articulation(
        "pump_stroke",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=-0.012,
            upper=0.0,
        ),
    )
    model.articulation(
        "nozzle_twist",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=nozzle,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
            0.5 * (aabb[0][2] + aabb[1][2]),
        )

    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    stem = object_model.get_part("plunger_stem")
    nozzle = object_model.get_part("nozzle_head")
    pump_stroke = object_model.get_articulation("pump_stroke")
    nozzle_twist = object_model.get_articulation("nozzle_twist")

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
        collar,
        bottle,
        contact_tol=0.001,
        name="collar_is_seated_on_the_bottle_neck",
    )
    ctx.expect_contact(
        stem,
        collar,
        contact_tol=0.001,
        name="stem_is_guided_by_the_fixed_collar",
    )
    ctx.expect_contact(
        nozzle,
        stem,
        contact_tol=0.001,
        name="nozzle_is_clipped_onto_the_stem",
    )
    ctx.expect_within(
        stem,
        collar,
        axes="xy",
        margin=0.0,
        name="stem_stays_centered_in_the_collar",
    )
    ctx.expect_gap(
        nozzle,
        collar,
        axis="z",
        min_gap=0.015,
        name="nozzle_sits_above_the_collar",
    )

    rest_stem_pos = ctx.part_world_position(stem)
    rest_nozzle_pos = ctx.part_world_position(nozzle)
    rest_spout_aabb = ctx.part_element_world_aabb(nozzle, elem="spout")
    assert rest_stem_pos is not None
    assert rest_nozzle_pos is not None
    assert rest_spout_aabb is not None
    rest_spout_center = aabb_center(rest_spout_aabb)

    ctx.check(
        "spout_points_forward_in_open_position",
        rest_spout_center[0] > 0.045 and abs(rest_spout_center[1]) < 0.006,
        (
            f"Expected forward-facing spout, got center "
            f"{tuple(round(v, 4) for v in rest_spout_center)}"
        ),
    )

    with ctx.pose({pump_stroke: -0.010}):
        pressed_stem_pos = ctx.part_world_position(stem)
        pressed_nozzle_pos = ctx.part_world_position(nozzle)
        assert pressed_stem_pos is not None
        assert pressed_nozzle_pos is not None

        ctx.expect_within(
            stem,
            collar,
            axes="xy",
            margin=0.0,
            name="stem_remains_guided_while_pressed",
        )
        ctx.expect_contact(
            nozzle,
            stem,
            contact_tol=0.001,
            name="nozzle_remains_clipped_while_pressed",
        )
        ctx.expect_gap(
            nozzle,
            collar,
            axis="z",
            min_gap=0.006,
            name="pressed_nozzle_still_clears_the_collar",
        )
        ctx.check(
            "pump_stroke_lowers_the_stem",
            pressed_stem_pos[2] < rest_stem_pos[2] - 0.009,
            (
                f"Expected stem z to drop by about 10 mm, got "
                f"{pressed_stem_pos[2]:.4f} from {rest_stem_pos[2]:.4f}"
            ),
        )
        ctx.check(
            "pump_stroke_lowers_the_nozzle",
            pressed_nozzle_pos[2] < rest_nozzle_pos[2] - 0.009,
            (
                f"Expected nozzle z to drop by about 10 mm, got "
                f"{pressed_nozzle_pos[2]:.4f} from {rest_nozzle_pos[2]:.4f}"
            ),
        )

    with ctx.pose({nozzle_twist: math.radians(115.0)}):
        locked_nozzle_pos = ctx.part_world_position(nozzle)
        locked_spout_aabb = ctx.part_element_world_aabb(nozzle, elem="spout")
        assert locked_nozzle_pos is not None
        assert locked_spout_aabb is not None
        locked_spout_center = aabb_center(locked_spout_aabb)

        ctx.expect_contact(
            nozzle,
            stem,
            contact_tol=0.001,
            name="nozzle_stays_attached_in_locked_rotation",
        )
        ctx.check(
            "nozzle_twist_keeps_the_joint_height_constant",
            abs(locked_nozzle_pos[2] - rest_nozzle_pos[2]) < 1e-6,
            (
                f"Expected nozzle origin height to stay fixed, got "
                f"{locked_nozzle_pos[2]:.6f} vs {rest_nozzle_pos[2]:.6f}"
            ),
        )
        ctx.check(
            "nozzle_twist_swings_the_spout_sideways",
            locked_spout_center[1] > rest_spout_center[1] + 0.030
            and locked_spout_center[0] < rest_spout_center[0] - 0.040,
            (
                f"Expected rotated spout center to swing around stem, got "
                f"rest={tuple(round(v, 4) for v in rest_spout_center)} "
                f"locked={tuple(round(v, 4) for v in locked_spout_center)}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
