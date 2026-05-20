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
    mesh_from_geometry,
)

_BUILD_SERIAL = 0


def build_object_model() -> ArticulatedObject:
    global _BUILD_SERIAL
    _BUILD_SERIAL += 1
    build_tag = f"b{_BUILD_SERIAL}"

    model = ArticulatedObject(name="selfie_ring_light_floor_stand")

    def circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
        return [
            (
                radius * math.cos((2.0 * math.pi * i) / segments),
                radius * math.sin((2.0 * math.pi * i) / segments),
            )
            for i in range(segments)
        ]

    def tube_shell(outer_radius: float, inner_radius: float, height: float, name: str):
        geom = LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, height)],
            [(inner_radius, 0.0), (inner_radius, height)],
            segments=48,
        )
        return mesh_from_geometry(geom, f"{name}_{build_tag}")

    def annulus_slab(
        outer_radius: float,
        inner_radius: float,
        depth: float,
        name: str,
    ):
        geom = ExtrudeWithHolesGeometry(
            circle_profile(outer_radius, segments=80),
            [list(reversed(circle_profile(inner_radius, segments=80)))],
            depth,
            center=True,
        )
        geom.rotate_y(math.pi / 2.0)
        return mesh_from_geometry(geom, f"{name}_{build_tag}")

    model.material("powder_black", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("diffuser_white", rgba=(0.96, 0.96, 0.94, 1.0))
    model.material("rubber_dark", rgba=(0.07, 0.07, 0.08, 1.0))

    stand_base = model.part("stand_base")
    stand_base.visual(
        Cylinder(radius=0.16, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="powder_black",
        name="weighted_base",
    )
    stand_base.visual(
        Cylinder(radius=0.145, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="rubber_dark",
        name="base_pad",
    )
    stand_base.visual(
        Cylinder(radius=0.05, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="powder_black",
        name="base_hub",
    )
    stand_base.visual(
        tube_shell(0.0185, 0.0140, 0.72, "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="silver",
        name="outer_sleeve",
    )
    stand_base.visual(
        tube_shell(0.024, 0.0140, 0.05, "clamp_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
        material="powder_black",
        name="clamp_collar",
    )
    stand_base.visual(
        Cylinder(radius=0.008, length=0.02),
        origin=Origin(xyz=(0.034, 0.0, 0.795), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="powder_black",
        name="lock_knob",
    )
    stand_base.inertial = Inertial.from_geometry(
        Box((0.32, 0.32, 1.10)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.014, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material="silver",
        name="inner_tube",
    )
    mast.visual(
        Box((0.02, 0.11, 0.16)),
        origin=Origin(xyz=(0.012, 0.0, 0.39)),
        material="powder_black",
        name="head_carrier",
    )
    mast.visual(
        Box((0.018, 0.012, 0.08)),
        origin=Origin(xyz=(0.03, 0.056, 0.48)),
        material="powder_black",
        name="left_yoke_ear",
    )
    mast.visual(
        Box((0.018, 0.012, 0.08)),
        origin=Origin(xyz=(0.03, -0.056, 0.48)),
        material="powder_black",
        name="right_yoke_ear",
    )
    mast.visual(
        Box((0.02, 0.05, 0.11)),
        origin=Origin(xyz=(0.006, 0.0, 0.13)),
        material="powder_black",
        name="shelf_carrier",
    )
    mast.visual(
        Box((0.018, 0.075, 0.014)),
        origin=Origin(xyz=(0.014, 0.0, 0.18)),
        material="powder_black",
        name="shelf_hinge_block",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.08, 0.14, 0.74)),
        mass=1.4,
        origin=Origin(xyz=(-0.01, 0.0, 0.22)),
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        annulus_slab(0.17, 0.118, 0.035, "ring_housing"),
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        material="satin_black",
        name="ring_housing",
    )
    ring_head.visual(
        annulus_slab(0.17, 0.118, 0.008, "ring_diffuser"),
        origin=Origin(xyz=(0.0535, 0.0, 0.0)),
        material="diffuser_white",
        name="ring_diffuser",
    )
    ring_head.visual(
        Cylinder(radius=0.008, length=0.11),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="powder_black",
        name="pivot_trunnion",
    )
    ring_head.visual(
        Box((0.028, 0.05, 0.042)),
        origin=Origin(xyz=(0.02, 0.0, -0.002)),
        material="powder_black",
        name="pivot_boss",
    )
    ring_head.visual(
        Box((0.024, 0.05, 0.10)),
        origin=Origin(xyz=(0.036, 0.0, -0.07)),
        material="powder_black",
        name="lower_mount_bracket",
    )
    ring_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.35, 0.18)),
        mass=1.2,
        origin=Origin(xyz=(0.04, 0.0, -0.03)),
    )

    phone_shelf = model.part("phone_shelf")
    phone_shelf.visual(
        Cylinder(radius=0.007, length=0.072),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="powder_black",
        name="hinge_barrel",
    )
    phone_shelf.visual(
        Box((0.056, 0.06, 0.018)),
        origin=Origin(xyz=(0.032, 0.0, -0.01)),
        material="powder_black",
        name="shelf_arm",
    )
    phone_shelf.visual(
        Box((0.16, 0.09, 0.008)),
        origin=Origin(xyz=(0.114, 0.0, -0.013)),
        material="satin_black",
        name="shelf_tray",
    )
    phone_shelf.visual(
        Box((0.016, 0.09, 0.018)),
        origin=Origin(xyz=(0.192, 0.0, -0.005)),
        material="powder_black",
        name="shelf_lip",
    )
    phone_shelf.visual(
        Box((0.11, 0.055, 0.0015)),
        origin=Origin(xyz=(0.114, 0.0, -0.00875)),
        material="rubber_dark",
        name="shelf_pad",
    )
    phone_shelf.inertial = Inertial.from_geometry(
        Box((0.21, 0.10, 0.04)),
        mass=0.18,
        origin=Origin(xyz=(0.10, 0.0, -0.01)),
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=stand_base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.25,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=ring_head,
        origin=Origin(xyz=(0.03, 0.0, 0.48)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "shelf_hinge",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=phone_shelf,
        origin=Origin(xyz=(0.03, 0.0, 0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-1.30,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    stand_base = object_model.get_part("stand_base")
    mast = object_model.get_part("mast")
    ring_head = object_model.get_part("ring_head")
    phone_shelf = object_model.get_part("phone_shelf")

    mast_slide = object_model.get_articulation("mast_slide")
    head_tilt = object_model.get_articulation("head_tilt")
    shelf_hinge = object_model.get_articulation("shelf_hinge")

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

    ctx.expect_contact(mast, stand_base, contact_tol=0.001)
    ctx.expect_contact(ring_head, mast, contact_tol=0.001)
    ctx.expect_contact(phone_shelf, mast, contact_tol=0.001)
    ctx.expect_gap(ring_head, phone_shelf, axis="z", min_gap=0.02)
    ctx.expect_overlap(phone_shelf, ring_head, axes="y", min_overlap=0.08)

    def span(aabb, axis: int) -> float:
        return aabb[1][axis] - aabb[0][axis]

    def center_from_aabb(aabb) -> tuple[float, float, float]:
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
            0.5 * (aabb[0][2] + aabb[1][2]),
        )

    rest_head_pos = ctx.part_world_position(ring_head)
    rest_shelf_pos = ctx.part_world_position(phone_shelf)
    with ctx.pose({mast_slide: 0.18}):
        raised_head_pos = ctx.part_world_position(ring_head)
        raised_shelf_pos = ctx.part_world_position(phone_shelf)
        raised_ok = (
            rest_head_pos is not None
            and rest_shelf_pos is not None
            and raised_head_pos is not None
            and raised_shelf_pos is not None
            and raised_head_pos[2] > rest_head_pos[2] + 0.17
            and raised_shelf_pos[2] > rest_shelf_pos[2] + 0.17
        )
        ctx.check(
            "mast_slide_raises_head_and_shelf",
            raised_ok,
            details="Prismatic mast should lift both the ring head and phone shelf together.",
        )

    rest_ring_aabb = ctx.part_element_world_aabb(ring_head, elem="ring_housing")
    with ctx.pose({head_tilt: 0.65}):
        tilted_ring_aabb = ctx.part_element_world_aabb(ring_head, elem="ring_housing")
        tilt_ok = (
            rest_ring_aabb is not None
            and tilted_ring_aabb is not None
            and span(tilted_ring_aabb, 0) > span(rest_ring_aabb, 0) + 0.10
        )
        ctx.check(
            "head_tilt_changes_ring_projection",
            tilt_ok,
            details="Tilting the head about the yoke's horizontal axis should increase the ring's x projection.",
        )
        ctx.expect_contact(ring_head, mast, contact_tol=0.001)

    rest_lip_aabb = ctx.part_element_world_aabb(phone_shelf, elem="shelf_lip")
    with ctx.pose({shelf_hinge: -1.20}):
        folded_lip_aabb = ctx.part_element_world_aabb(phone_shelf, elem="shelf_lip")
        folded_ok = (
            rest_lip_aabb is not None
            and folded_lip_aabb is not None
            and center_from_aabb(folded_lip_aabb)[2] > center_from_aabb(rest_lip_aabb)[2] + 0.11
            and center_from_aabb(folded_lip_aabb)[0] < center_from_aabb(rest_lip_aabb)[0] - 0.08
        )
        ctx.check(
            "phone_shelf_folds_on_its_own_hinge",
            folded_ok,
            details="The shelf lip should swing up and back when the dedicated shelf hinge is rotated.",
        )
        ctx.expect_contact(phone_shelf, mast, contact_tol=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
