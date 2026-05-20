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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_vent_tower_utility", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.30, 0.32, 0.34, 1.0))
    liner_black = model.material("liner_black", rgba=(0.14, 0.14, 0.15, 1.0))
    fastener_metal = model.material("fastener_metal", rgba=(0.72, 0.74, 0.78, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body_w = 0.32
    body_d = 0.22
    body_h = 0.34
    wall_t = 0.024
    base_w = 0.42
    base_d = 0.34
    base_t = 0.018
    top_z = base_t + body_h
    front_face_y = body_d / 2.0
    inner_w = body_w - 2.0 * wall_t

    tower = model.part("vent_tower")
    tower.visual(
        Box((base_w, base_d, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=painted_steel,
        name="mounting_flange",
    )
    tower.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(-(body_w - wall_t) / 2.0, 0.0, base_t + body_h / 2.0)),
        material=painted_steel,
        name="left_wall",
    )
    tower.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=((body_w - wall_t) / 2.0, 0.0, base_t + body_h / 2.0)),
        material=painted_steel,
        name="right_wall",
    )
    tower.visual(
        Box((inner_w, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -(body_d - wall_t) / 2.0, base_t + body_h / 2.0)),
        material=painted_steel,
        name="rear_wall",
    )
    tower.visual(
        Box((inner_w, wall_t, 0.10)),
        origin=Origin(xyz=(0.0, (body_d - wall_t) / 2.0, base_t + 0.05)),
        material=painted_steel,
        name="front_sill_panel",
    )
    tower.visual(
        Box((inner_w, wall_t + 0.012, 0.018)),
        origin=Origin(
            xyz=(0.0, front_face_y - wall_t / 2.0 + 0.006, base_t + 0.10 + 0.009),
        ),
        material=painted_steel,
        name="outlet_sill",
    )
    tower.visual(
        Box((inner_w, wall_t + 0.012, 0.026)),
        origin=Origin(
            xyz=(0.0, front_face_y - wall_t / 2.0 + 0.006, base_t + 0.10 + 0.13 + 0.013),
        ),
        material=painted_steel,
        name="outlet_lintel",
    )
    jamb_w = (inner_w - 0.22) / 2.0
    jamb_z = base_t + 0.10 + 0.13 / 2.0 + 0.009
    jamb_h = 0.13 + 0.018
    for side, sign in (("left", -1.0), ("right", 1.0)):
        tower.visual(
            Box((jamb_w, wall_t + 0.012, jamb_h)),
            origin=Origin(
                xyz=(
                    sign * (0.22 / 2.0 + jamb_w / 2.0),
                    front_face_y - wall_t / 2.0 + 0.006,
                    jamb_z,
                )
            ),
            material=painted_steel,
            name=f"outlet_{side}_jamb",
        )

    tower.visual(
        Box((inner_w, 0.050, wall_t)),
        origin=Origin(xyz=(0.0, -0.085, top_z - wall_t / 2.0)),
        material=painted_steel,
        name="hinge_bridge",
    )
    tower.visual(
        Box((0.018, 0.17, 0.10)),
        origin=Origin(xyz=(-body_w / 2.0 - 0.009, 0.008, base_t + 0.05)),
        material=painted_steel,
        name="left_side_rib",
    )
    tower.visual(
        Box((0.018, 0.17, 0.10)),
        origin=Origin(xyz=(body_w / 2.0 + 0.009, 0.008, base_t + 0.05)),
        material=painted_steel,
        name="right_side_rib",
    )
    tower.visual(
        Box((0.20, 0.010, 0.10)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 - 0.005, base_t + 0.05)),
        material=painted_steel,
        name="rear_stiffener",
    )
    tower.visual(
        Box((inner_w - 0.04, wall_t * 0.6, 0.12)),
        origin=Origin(xyz=(0.0, -(body_d / 2.0 - wall_t * 0.7), base_t + 0.17)),
        material=liner_black,
        name="interior_shadow_baffle",
    )

    hinge_axis_y = -0.102
    hinge_axis_z = top_z + 0.008
    tower.visual(
        Box((0.075, 0.018, 0.022)),
        origin=Origin(xyz=(-0.0875, -0.101, hinge_axis_z - 0.022 / 2.0 - 0.011)),
        material=painted_steel,
        name="hinge_left_bracket",
    )
    tower.visual(
        Box((0.075, 0.018, 0.022)),
        origin=Origin(xyz=(0.0875, -0.101, hinge_axis_z - 0.022 / 2.0 - 0.011)),
        material=painted_steel,
        name="hinge_right_bracket",
    )
    tower.visual(
        Cylinder(radius=0.011, length=0.075),
        origin=Origin(xyz=(-0.0875, hinge_axis_y, hinge_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="tower_hinge_left",
    )
    tower.visual(
        Cylinder(radius=0.011, length=0.075),
        origin=Origin(xyz=(0.0875, hinge_axis_y, hinge_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="tower_hinge_right",
    )
    tower.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(-0.131, hinge_axis_y, hinge_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fastener_metal,
        name="hinge_pin_end_left",
    )
    tower.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.131, hinge_axis_y, hinge_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fastener_metal,
        name="hinge_pin_end_right",
    )

    def add_front_bolt(name: str, x: float, z: float) -> None:
        tower.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(
                xyz=(x, front_face_y + 0.003, z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=fastener_metal,
            name=name,
        )

    add_front_bolt("lintel_bolt_left", -0.103, base_t + 0.10 + 0.13 + 0.013)
    add_front_bolt("lintel_bolt_right", 0.103, base_t + 0.10 + 0.13 + 0.013)
    add_front_bolt("sill_bolt_left", -0.103, base_t + 0.10 + 0.009)
    add_front_bolt("sill_bolt_right", 0.103, base_t + 0.10 + 0.009)

    for side, sign in (("left", -1.0), ("right", 1.0)):
        for idx, z in enumerate((hinge_axis_z - 0.030, hinge_axis_z - 0.012), start=1):
            tower.visual(
                Cylinder(radius=0.0045, length=0.006),
                origin=Origin(
                    xyz=(sign * 0.0875, -0.091, z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=fastener_metal,
                name=f"hinge_{side}_bolt_{idx}",
            )

    tower.inertial = Inertial.from_geometry(
        Box((base_w, base_d, top_z)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, top_z / 2.0)),
    )

    flap = model.part("weather_flap")
    flap.visual(
        Box((0.34, 0.228, 0.012)),
        origin=Origin(xyz=(0.0, 0.126, 0.004)),
        material=painted_steel,
        name="cover_shell",
    )
    flap.visual(
        Box((0.012, 0.228, 0.236)),
        origin=Origin(xyz=(-0.166, 0.114, -0.114)),
        material=painted_steel,
        name="left_cheek",
    )
    flap.visual(
        Box((0.012, 0.228, 0.236)),
        origin=Origin(xyz=(0.166, 0.114, -0.114)),
        material=painted_steel,
        name="right_cheek",
    )
    flap.visual(
        Box((0.216, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.221, -0.078)),
        material=painted_steel,
        name="drip_lip",
    )
    flap.visual(
        Box((0.020, 0.150, 0.090)),
        origin=Origin(xyz=(-0.096, 0.112, -0.028)),
        material=painted_steel,
        name="left_under_rib",
    )
    flap.visual(
        Box((0.020, 0.150, 0.090)),
        origin=Origin(xyz=(0.096, 0.112, -0.028)),
        material=painted_steel,
        name="right_under_rib",
    )
    flap.visual(
        Box((0.090, 0.032, 0.022)),
        origin=Origin(xyz=(0.0, 0.016, 0.000)),
        material=painted_steel,
        name="rear_hinge_strap",
    )
    flap.visual(
        Box((0.308, 0.014, 0.236)),
        origin=Origin(xyz=(0.0, 0.235, -0.114)),
        material=painted_steel,
        name="front_closure_panel",
    )
    flap.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=liner_black,
        name="flap_barrel",
    )
    flap.visual(
        Box((0.230, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.230, -0.224)),
        material=gasket_black,
        name="seal_strip",
    )

    def add_cover_bolt(name: str, x: float, y: float) -> None:
        flap.visual(
            Cylinder(radius=0.005, length=0.016),
            origin=Origin(xyz=(x, y, 0.006)),
            material=fastener_metal,
            name=name,
        )

    add_cover_bolt("cover_bolt_left_front", -0.110, 0.170)
    add_cover_bolt("cover_bolt_right_front", 0.110, 0.170)
    add_cover_bolt("cover_bolt_left_rear", -0.110, 0.080)
    add_cover_bolt("cover_bolt_right_rear", 0.110, 0.080)

    flap.inertial = Inertial.from_geometry(
        Box((0.35, 0.25, 0.24)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.12, -0.11)),
    )

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("vent_tower")
    flap = object_model.get_part("weather_flap")
    tower_to_flap = object_model.get_articulation("tower_to_flap")
    tower_hinge_left = tower.get_visual("tower_hinge_left")
    tower_hinge_right = tower.get_visual("tower_hinge_right")
    outlet_lintel = tower.get_visual("outlet_lintel")
    outlet_sill = tower.get_visual("outlet_sill")
    hinge_bridge = tower.get_visual("hinge_bridge")
    cover_shell = flap.get_visual("cover_shell")
    flap_barrel = flap.get_visual("flap_barrel")
    drip_lip = flap.get_visual("drip_lip")
    seal_strip = flap.get_visual("seal_strip")

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

    ctx.expect_contact(
        flap,
        tower,
        elem_a=flap_barrel,
        elem_b=tower_hinge_left,
        name="left_hinge_barrels_meet",
    )
    ctx.expect_contact(
        flap,
        tower,
        elem_a=flap_barrel,
        elem_b=tower_hinge_right,
        name="right_hinge_barrels_meet",
    )
    ctx.expect_overlap(
        flap,
        tower,
        axes="x",
        elem_a=cover_shell,
        elem_b=outlet_lintel,
        min_overlap=0.26,
        name="flap_covers_outlet_width",
    )
    ctx.expect_gap(
        flap,
        tower,
        axis="z",
        positive_elem=cover_shell,
        negative_elem=hinge_bridge,
        min_gap=0.0,
        max_gap=0.030,
        name="closed_cover_sits_just_above_hinge_bridge",
    )
    ctx.expect_gap(
        flap,
        tower,
        axis="z",
        positive_elem=drip_lip,
        negative_elem=outlet_lintel,
        min_gap=0.0,
        max_gap=0.060,
        name="closed_drip_lip_shelters_opening",
    )
    ctx.expect_gap(
        flap,
        tower,
        axis="z",
        positive_elem=seal_strip,
        negative_elem=outlet_sill,
        min_gap=-0.002,
        max_gap=0.045,
        name="seal_strip_stays_near_outlet_plane",
    )

    tower_aabb = ctx.part_world_aabb(tower)
    ctx.check(
        "tower_has_rugged_aspect_ratio",
        tower_aabb is not None
        and 0.30 <= tower_aabb[1][0] - tower_aabb[0][0] <= 0.46
        and 0.22 <= tower_aabb[1][1] - tower_aabb[0][1] <= 0.36
        and 0.34 <= tower_aabb[1][2] - tower_aabb[0][2] <= 0.40,
        details=f"tower aabb={tower_aabb}",
    )

    flap_rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({tower_to_flap: math.radians(62.0)}):
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            positive_elem=drip_lip,
            negative_elem=outlet_lintel,
            min_gap=0.090,
            name="open_flap_clears_lintel",
        )
        flap_open_aabb = ctx.part_world_aabb(flap)
        ctx.check(
            "flap_rises_when_opened",
            flap_rest_aabb is not None
            and flap_open_aabb is not None
            and flap_open_aabb[1][2] > flap_rest_aabb[1][2] + 0.12
            and flap_open_aabb[0][2] > flap_rest_aabb[0][2] + 0.10,
            details=f"rest={flap_rest_aabb}, open={flap_open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
