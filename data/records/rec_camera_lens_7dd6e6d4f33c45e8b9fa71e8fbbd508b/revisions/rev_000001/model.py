from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_mesh(name: str, outer_profile, inner_profile, *, segments: int = 88):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telephoto_zoom_lens")

    barrel_white = model.material("barrel_white", rgba=(0.82, 0.83, 0.80, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.16, 0.22, 0.26, 0.55))

    barrel = model.part("barrel")
    barrel.visual(
        _shell_mesh(
            "lens_barrel_shell",
            [
                (0.051, 0.000),
                (0.054, 0.010),
                (0.058, 0.024),
                (0.058, 0.100),
                (0.0555, 0.118),
                (0.0555, 0.210),
                (0.0545, 0.238),
                (0.0545, 0.292),
                (0.0575, 0.305),
                (0.059, 0.318),
            ],
            [
                (0.031, 0.000),
                (0.040, 0.010),
                (0.0495, 0.024),
                (0.0505, 0.100),
                (0.0505, 0.210),
                (0.0498, 0.292),
                (0.0475, 0.305),
                (0.0460, 0.318),
            ],
        ),
        material=barrel_white,
        name="barrel_shell",
    )
    barrel.visual(
        _shell_mesh(
            "rear_mount_ring",
            [
                (0.043, -0.010),
                (0.047, -0.003),
                (0.047, 0.010),
            ],
            [
                (0.024, -0.010),
                (0.024, 0.010),
            ],
        ),
        material=graphite,
        name="rear_mount",
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        _shell_mesh(
            "inner_barrel_shell",
            [
                (0.0475, -0.080),
                (0.0485, -0.030),
                (0.0485, 0.085),
                (0.0500, 0.120),
                (0.0520, 0.138),
                (0.0530, 0.150),
            ],
            [
                (0.0430, -0.080),
                (0.0430, 0.115),
                (0.0410, 0.150),
            ],
        ),
        material=graphite,
        name="inner_shell",
    )
    inner_barrel.visual(
        Cylinder(radius=0.0417, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=coated_glass,
        name="front_glass",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _shell_mesh(
            "focus_ring_shell",
            [
                (0.0620, -0.030),
                (0.0640, -0.024),
                (0.0618, -0.018),
                (0.0642, -0.012),
                (0.0618, -0.006),
                (0.0642, 0.000),
                (0.0618, 0.006),
                (0.0642, 0.012),
                (0.0618, 0.018),
                (0.0640, 0.024),
                (0.0620, 0.030),
            ],
            [
                (0.0562, -0.031),
                (0.0562, 0.031),
            ],
        ),
        material=rubber_black,
        name="focus_shell",
    )
    focus_ring.visual(
        Box((0.012, 0.0022, 0.016)),
        origin=Origin(xyz=(0.0, 0.0566, 0.0)),
        material=rubber_black,
        name="bearing_pad_0",
    )
    focus_ring.visual(
        Box((0.012, 0.0022, 0.016)),
        origin=Origin(xyz=(0.0, -0.0566, 0.0)),
        material=rubber_black,
        name="bearing_pad_1",
    )
    focus_ring.visual(
        Box((0.0022, 0.012, 0.016)),
        origin=Origin(xyz=(0.0566, 0.0, 0.0)),
        material=rubber_black,
        name="bearing_pad_2",
    )
    focus_ring.visual(
        Box((0.0022, 0.012, 0.016)),
        origin=Origin(xyz=(-0.0566, 0.0, 0.0)),
        material=rubber_black,
        name="bearing_pad_3",
    )

    model.articulation(
        "barrel_to_inner_barrel",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=inner_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.12,
            lower=0.0,
            upper=0.085,
        ),
    )
    model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=8.0,
        ),
    )

    collar = model.part("collar")
    collar.visual(
        _shell_mesh(
            "tripod_collar_shell",
            [
                (0.0645, -0.025),
                (0.0665, -0.018),
                (0.0665, 0.018),
                (0.0645, 0.025),
            ],
            [
                (0.0606, -0.026),
                (0.0606, 0.026),
            ],
        ),
        material=barrel_white,
        name="collar_shell",
    )
    collar.visual(
        Box((0.062, 0.020, 0.054)),
        origin=Origin(xyz=(0.0, -0.064, 0.0)),
        material=barrel_white,
        name="clamp_block",
    )
    collar.visual(
        Box((0.040, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, -0.085, 0.0)),
        material=barrel_white,
        name="foot_stem",
    )
    collar.visual(
        Box((0.066, 0.014, 0.122)),
        origin=Origin(xyz=(0.0, -0.101, 0.0)),
        material=graphite,
        name="foot_pad",
    )
    collar.visual(
        Box((0.012, 0.0026, 0.016)),
        origin=Origin(xyz=(0.0, 0.0593, -0.010)),
        material=graphite,
        name="collar_pad_0",
    )
    collar.visual(
        Box((0.012, 0.0026, 0.016)),
        origin=Origin(xyz=(0.0, -0.0593, 0.010)),
        material=graphite,
        name="collar_pad_1",
    )
    collar.visual(
        Box((0.0026, 0.012, 0.016)),
        origin=Origin(xyz=(0.0593, 0.0, 0.010)),
        material=graphite,
        name="collar_pad_2",
    )
    collar.visual(
        Box((0.0026, 0.012, 0.016)),
        origin=Origin(xyz=(-0.0593, 0.0, -0.010)),
        material=graphite,
        name="collar_pad_3",
    )

    lock_knob = model.part("lock_knob")
    lock_knob.visual(
        Cylinder(radius=0.0042, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="shaft",
    )
    lock_knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="knob_hub",
    )
    lock_knob.visual(
        Cylinder(radius=0.015, length=0.008),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="knob_cap",
    )

    model.articulation(
        "barrel_to_collar",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
        ),
    )
    model.articulation(
        "collar_to_lock_knob",
        ArticulationType.CONTINUOUS,
        parent=collar,
        child=lock_knob,
        origin=Origin(xyz=(0.031, -0.064, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=12.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    inner_barrel = object_model.get_part("inner_barrel")
    focus_ring = object_model.get_part("focus_ring")
    collar = object_model.get_part("collar")
    lock_knob = object_model.get_part("lock_knob")
    inner_slide = object_model.get_articulation("barrel_to_inner_barrel")
    focus_spin = object_model.get_articulation("barrel_to_focus_ring")
    collar_spin = object_model.get_articulation("barrel_to_collar")
    knob_spin = object_model.get_articulation("collar_to_lock_knob")

    ctx.allow_overlap(
        barrel,
        inner_barrel,
        elem_a="barrel_shell",
        elem_b="inner_shell",
        reason=(
            "The inner tube is intentionally modeled as a retained telescoping member "
            "inside the hollow outer sleeve, and the compiled mesh-shell pair still "
            "classifies this nested sleeve fit as overlap."
        ),
    )

    ctx.expect_within(
        inner_barrel,
        barrel,
        axes="xy",
        inner_elem="inner_shell",
        outer_elem="barrel_shell",
        margin=0.010,
        name="inner barrel stays concentric inside the outer sleeve",
    )
    ctx.expect_overlap(
        inner_barrel,
        barrel,
        axes="z",
        elem_a="inner_shell",
        elem_b="barrel_shell",
        min_overlap=0.090,
        name="collapsed inner barrel remains inserted in the outer sleeve",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="xy",
        elem_a="focus_shell",
        elem_b="barrel_shell",
        min_overlap=0.105,
        name="focus ring wraps around the outer barrel",
    )
    ctx.expect_contact(
        focus_ring,
        barrel,
        elem_a="bearing_pad_0",
        elem_b="barrel_shell",
        contact_tol=0.0002,
        name="focus ring rides on the barrel bearing land",
    )
    ctx.expect_overlap(
        collar,
        barrel,
        axes="xy",
        elem_a="collar_shell",
        elem_b="barrel_shell",
        min_overlap=0.118,
        name="tripod collar wraps around the main barrel",
    )
    ctx.expect_contact(
        collar,
        barrel,
        elem_a="collar_pad_0",
        elem_b="barrel_shell",
        contact_tol=0.0003,
        name="tripod collar rides on the barrel bearing pads",
    )
    ctx.expect_gap(
        barrel,
        collar,
        axis="y",
        positive_elem="barrel_shell",
        negative_elem="foot_pad",
        min_gap=0.028,
        name="tripod foot hangs below the barrel centerline",
    )
    ctx.expect_contact(
        lock_knob,
        collar,
        elem_a="shaft",
        elem_b="clamp_block",
        contact_tol=0.0002,
        name="lock knob shaft seats against the collar clamp block",
    )

    rest_pos = ctx.part_world_position(inner_barrel)
    with ctx.pose({inner_slide: 0.085}):
        ctx.expect_within(
            inner_barrel,
            barrel,
            axes="xy",
            inner_elem="inner_shell",
            outer_elem="barrel_shell",
            margin=0.010,
            name="extended inner barrel stays centered in the outer sleeve",
        )
        ctx.expect_overlap(
            inner_barrel,
            barrel,
            axes="z",
            elem_a="inner_shell",
            elem_b="barrel_shell",
            min_overlap=0.085,
            name="extended inner barrel keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(inner_barrel)

    ctx.check(
        "inner barrel extends forward along the optical axis",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.05,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "focus ring uses continuous axial rotation",
        focus_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(focus_spin.axis) == (0.0, 0.0, 1.0)
        and focus_spin.motion_limits is not None
        and focus_spin.motion_limits.lower is None
        and focus_spin.motion_limits.upper is None,
        details=(
            f"type={focus_spin.articulation_type}, axis={focus_spin.axis}, "
            f"limits={focus_spin.motion_limits}"
        ),
    )
    ctx.check(
        "tripod collar rotates continuously about the optical axis",
        collar_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(collar_spin.axis) == (0.0, 0.0, 1.0)
        and collar_spin.motion_limits is not None
        and collar_spin.motion_limits.lower is None
        and collar_spin.motion_limits.upper is None,
        details=(
            f"type={collar_spin.articulation_type}, axis={collar_spin.axis}, "
            f"limits={collar_spin.motion_limits}"
        ),
    )
    ctx.check(
        "lock knob rotates continuously on its threaded shaft",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_spin.axis) == (1.0, 0.0, 0.0)
        and knob_spin.motion_limits is not None
        and knob_spin.motion_limits.lower is None
        and knob_spin.motion_limits.upper is None,
        details=(
            f"type={knob_spin.articulation_type}, axis={knob_spin.axis}, "
            f"limits={knob_spin.motion_limits}"
        ),
    )

    foot_rest = _aabb_center(ctx.part_element_world_aabb(collar, elem="foot_pad"))
    with ctx.pose({collar_spin: math.pi / 2.0}):
        foot_rotated = _aabb_center(ctx.part_element_world_aabb(collar, elem="foot_pad"))

    ctx.check(
        "tripod foot orbits around the barrel when the collar rotates",
        foot_rest is not None
        and foot_rotated is not None
        and abs(foot_rotated[0] - foot_rest[0]) > 0.08
        and abs(foot_rotated[1] - foot_rest[1]) > 0.08,
        details=f"rest={foot_rest}, rotated={foot_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
