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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def build_bottle_shell():
        outer_profile = [
            (0.0, 0.0),
            (0.031, 0.0),
            (0.035, 0.004),
            (0.038, 0.016),
            (0.038, 0.112),
            (0.036, 0.128),
            (0.031, 0.141),
            (0.0240, 0.1455),
            (0.0205, 0.1470),
            (0.0188, 0.1480),
            (0.0, 0.1480),
        ]
        inner_profile = [
            (0.0, 0.004),
            (0.0315, 0.007),
            (0.0352, 0.018),
            (0.0352, 0.110),
            (0.0335, 0.126),
            (0.0290, 0.139),
            (0.0215, 0.1455),
            (0.0138, 0.1480),
            (0.0, 0.1480),
        ]
        return boolean_difference(
            LatheGeometry(outer_profile, segments=80),
            LatheGeometry(inner_profile, segments=80),
        )

    def build_collar_shell():
        outer_profile = [
            (0.0, 0.0),
            (0.027, 0.0),
            (0.027, 0.010),
            (0.0245, 0.010),
            (0.0245, 0.021),
            (0.0210, 0.026),
            (0.0, 0.026),
        ]
        collar_outer = LatheGeometry(outer_profile, segments=72)
        lower_neck_cavity = CylinderGeometry(radius=0.0196, height=0.018).translate(0.0, 0.0, 0.009)
        stem_guide_hole = CylinderGeometry(radius=0.0062, height=0.032).translate(0.0, 0.0, 0.013)
        return boolean_difference(
            boolean_difference(collar_outer, lower_neck_cavity),
            stem_guide_hole,
        )

    def build_spout_body():
        def yz_section(width_y: float, height_z: float, radius: float, x: float, z_mid: float):
            return [
                (x, y, z_mid + z)
                for z, y in rounded_rect_profile(height_z, width_y, radius, corner_segments=8)
            ]

        return section_loft(
            [
                yz_section(0.019, 0.009, 0.0030, -0.003, 0.011),
                yz_section(0.024, 0.008, 0.0032, 0.010, 0.013),
                yz_section(0.018, 0.006, 0.0025, 0.024, 0.011),
            ]
        )

    model = ArticulatedObject(name="soap_dispenser_pump_bottle", assets=ASSETS)

    bottle_plastic = model.material("bottle_plastic", rgba=(0.88, 0.73, 0.47, 0.72))
    pump_black = model.material("pump_black", rgba=(0.13, 0.13, 0.14, 1.0))
    stem_ivory = model.material("stem_ivory", rgba=(0.93, 0.92, 0.88, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        save_mesh("bottle_shell.obj", build_bottle_shell()),
        material=bottle_plastic,
        name="bottle_body_shell",
    )
    bottle.visual(
        Cylinder(radius=0.0245, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.149)),
        material=bottle_plastic,
        name="shoulder_lip",
    )
    bottle.visual(
        Cylinder(radius=0.0178, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=bottle_plastic,
        name="neck_core",
    )
    bottle.visual(
        Cylinder(radius=0.0187, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.1516)),
        material=bottle_plastic,
        name="neck_thread_lower",
    )
    bottle.visual(
        Cylinder(radius=0.0187, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.1550)),
        material=bottle_plastic,
        name="neck_thread_upper",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.166),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
    )

    collar = model.part("pump_collar")
    collar.visual(
        save_mesh("pump_collar.obj", build_collar_shell()),
        material=pump_black,
        name="collar_shell",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.027, length=0.026),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    plunger = model.part("plunger_stem")
    plunger.visual(
        Cylinder(radius=0.0044, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=stem_ivory,
        name="stem_shaft",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0044, length=0.055),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    spout = model.part("spout_head")
    spout.visual(
        Cylinder(radius=0.0105, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=pump_black,
        name="hub",
    )
    spout.visual(
        save_mesh("spout_body.obj", build_spout_body()),
        material=pump_black,
        name="top_pad",
    )
    spout.visual(
        Cylinder(radius=0.0038, length=0.032),
        origin=Origin(xyz=(0.022, 0.0, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_black,
        name="nozzle_tube",
    )
    spout.visual(
        Cylinder(radius=0.0024, length=0.008),
        origin=Origin(xyz=(0.037, 0.0, 0.001)),
        material=pump_black,
        name="outlet_tip",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.045, 0.024, 0.020)),
        mass=0.03,
        origin=Origin(xyz=(0.013, 0.0, 0.010)),
    )

    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
    )
    model.articulation(
        "collar_to_plunger",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=-0.010,
            upper=0.0,
        ),
    )
    model.articulation(
        "plunger_to_spout",
        ArticulationType.CONTINUOUS,
        parent=plunger,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("pump_collar")
    plunger = object_model.get_part("plunger_stem")
    spout = object_model.get_part("spout_head")

    bottle_to_collar = object_model.get_articulation("bottle_to_collar")
    collar_to_plunger = object_model.get_articulation("collar_to_plunger")
    plunger_to_spout = object_model.get_articulation("plunger_to_spout")

    bottle_shell = bottle.get_visual("bottle_body_shell")
    collar_shell = collar.get_visual("collar_shell")
    stem_shaft = plunger.get_visual("stem_shaft")
    hub = spout.get_visual("hub")
    nozzle_tube = spout.get_visual("nozzle_tube")

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

    ctx.check("bottle exists", bottle is not None)
    ctx.check("pump collar exists", collar is not None)
    ctx.check("plunger stem exists", plunger is not None)
    ctx.check("spout head exists", spout is not None)
    ctx.check("fixed collar mount exists", bottle_to_collar is not None)
    ctx.check("plunger slider exists", collar_to_plunger is not None)
    ctx.check("spout rotation exists", plunger_to_spout is not None)

    ctx.expect_contact(collar, bottle)
    ctx.expect_overlap(collar, bottle, axes="xy", min_overlap=0.035)
    ctx.expect_origin_distance(collar, bottle, axes="xy", max_dist=1e-6)
    ctx.expect_origin_distance(plunger, collar, axes="xy", max_dist=1e-6)
    ctx.expect_contact(spout, plunger, elem_a=hub, elem_b=stem_shaft)
    ctx.expect_gap(spout, collar, axis="z", min_gap=0.024, max_gap=0.040)
    ctx.expect_overlap(plunger, collar, axes="xy", min_overlap=0.008)
    ctx.expect_overlap(spout, bottle, axes="xy", min_overlap=0.020)

    rest_plunger_pos = ctx.part_world_position(plunger)
    pressed_plunger_pos = None
    ctx.check("plunger rest position measurable", rest_plunger_pos is not None)
    if rest_plunger_pos is not None:
        with ctx.pose({collar_to_plunger: -0.009}):
            pressed_plunger_pos = ctx.part_world_position(plunger)
            ctx.expect_origin_distance(plunger, collar, axes="xy", max_dist=1e-6)
            ctx.expect_gap(spout, collar, axis="z", min_gap=0.015, max_gap=0.031)
            ctx.expect_contact(spout, plunger, elem_a=hub, elem_b=stem_shaft)
        ctx.check(
            "plunger translates downward by press travel",
            pressed_plunger_pos is not None and pressed_plunger_pos[2] < rest_plunger_pos[2] - 0.0085,
            details=""
            if pressed_plunger_pos is not None
            else "Pressed plunger world position was not measurable.",
        )

    rest_spout_aabb = ctx.part_world_aabb(spout)
    rotated_spout_aabb = None
    ctx.check("spout rest AABB measurable", rest_spout_aabb is not None)
    if rest_spout_aabb is not None:
        with ctx.pose({plunger_to_spout: math.pi / 2.0}):
            rotated_spout_aabb = ctx.part_world_aabb(spout)
            ctx.expect_contact(spout, plunger, elem_a=hub, elem_b=stem_shaft)
            ctx.expect_origin_distance(spout, plunger, axes="xy", max_dist=1e-6)

        rest_dx = rest_spout_aabb[1][0] - rest_spout_aabb[0][0]
        rest_dy = rest_spout_aabb[1][1] - rest_spout_aabb[0][1]
        ctx.check(
            "spout is asymmetric in rest pose",
            rest_dx > rest_dy + 0.010,
            details=f"Expected x-span > y-span for forward nozzle, got dx={rest_dx:.4f}, dy={rest_dy:.4f}.",
        )
        if rotated_spout_aabb is not None:
            rot_dx = rotated_spout_aabb[1][0] - rotated_spout_aabb[0][0]
            rot_dy = rotated_spout_aabb[1][1] - rotated_spout_aabb[0][1]
            ctx.check(
                "spout continuous rotation changes nozzle heading",
                rot_dy > rot_dx + 0.010,
                details=f"Expected y-span > x-span after quarter turn, got dx={rot_dx:.4f}, dy={rot_dy:.4f}.",
            )

    bottle_aabb = ctx.part_element_world_aabb(bottle, elem=bottle_shell)
    collar_aabb = ctx.part_element_world_aabb(collar, elem=collar_shell)
    stem_aabb = ctx.part_element_world_aabb(plunger, elem=stem_shaft)
    nozzle_aabb = ctx.part_element_world_aabb(spout, elem=nozzle_tube)
    ctx.check("bottle shell measurable", bottle_aabb is not None)
    ctx.check("collar shell measurable", collar_aabb is not None)
    ctx.check("stem shaft measurable", stem_aabb is not None)
    ctx.check("nozzle measurable", nozzle_aabb is not None)
    if bottle_aabb is not None and collar_aabb is not None and stem_aabb is not None and nozzle_aabb is not None:
        bottle_height = bottle_aabb[1][2] - bottle_aabb[0][2]
        collar_height = collar_aabb[1][2] - collar_aabb[0][2]
        stem_height = stem_aabb[1][2] - stem_aabb[0][2]
        nozzle_reach = nozzle_aabb[1][0] - nozzle_aabb[0][0]
        ctx.check(
            "bottle body is the dominant mass",
            bottle_height > stem_height * 2.5 and bottle_height > collar_height * 4.5,
            details=(
                f"Expected bottle height to dominate silhouette, got bottle={bottle_height:.4f}, "
                f"stem={stem_height:.4f}, collar={collar_height:.4f}."
            ),
        )
        ctx.check(
            "spout nozzle has short realistic reach",
            0.020 <= nozzle_reach <= 0.038,
            details=f"Expected nozzle reach in [0.020, 0.038], got {nozzle_reach:.4f}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
