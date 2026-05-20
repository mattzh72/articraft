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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="candy_vending_cabinet")

    cabinet_red = model.material("cabinet_red", rgba=(0.74, 0.16, 0.12, 1.0))
    powder_black = model.material("powder_black", rgba=(0.16, 0.17, 0.18, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.72, 0.86, 0.95, 0.35))
    knob_cream = model.material("knob_cream", rgba=(0.93, 0.88, 0.73, 1.0))
    zinc_gray = model.material("zinc_gray", rgba=(0.63, 0.65, 0.67, 1.0))

    cabinet_w = 0.40
    cabinet_d = 0.32
    cabinet_h = 0.68
    wall_t = 0.018
    front_y = cabinet_d / 2.0

    front_clear_w = 0.25
    front_clear_h = 0.32
    front_lower_h = 0.252
    front_header_h = 0.072
    side_inner_depth = cabinet_d - (2.0 * wall_t)
    side_margin = 0.032
    service_open_d = side_inner_depth - (2.0 * side_margin)
    service_bottom_z = 0.088
    service_open_h = 0.50
    service_top_h = (cabinet_h - wall_t) - (service_bottom_z + service_open_h)

    chute_w = 0.18
    chute_out = 0.09
    chute_side_t = 0.012
    chute_floor_t = 0.012
    chute_roof_t = 0.012
    chute_bottom_z = 0.045
    chute_open_h = 0.10
    dispense_flap_h = chute_open_h - chute_floor_t
    dispense_flap_t = 0.008
    dispense_open_w = chute_w - (2.0 * chute_side_t)

    knob_x = 0.118
    knob_z = 0.195
    bushing_len = 0.012

    body = model.part("cabinet_body")
    body.visual(
        Box((cabinet_w - (2.0 * wall_t), cabinet_d - (2.0 * wall_t), wall_t)),
        origin=Origin(xyz=(0.0, 0.0, wall_t / 2.0)),
        material=powder_black,
        name="cabinet_floor",
    )
    body.visual(
        Box((cabinet_w - (2.0 * wall_t), cabinet_d - (2.0 * wall_t), wall_t)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_h - (wall_t / 2.0))),
        material=cabinet_red,
        name="cabinet_roof",
    )
    body.visual(
        Box((wall_t, cabinet_d - (2.0 * wall_t), cabinet_h - (2.0 * wall_t))),
        origin=Origin(xyz=(-(cabinet_w / 2.0) + (wall_t / 2.0), 0.0, cabinet_h / 2.0)),
        material=cabinet_red,
        name="left_side_panel",
    )
    body.visual(
        Box((cabinet_w - (2.0 * wall_t), wall_t, cabinet_h - (2.0 * wall_t))),
        origin=Origin(xyz=(0.0, -(cabinet_d / 2.0) + (wall_t / 2.0), cabinet_h / 2.0)),
        material=cabinet_red,
        name="back_panel",
    )
    body.visual(
        Box((wall_t, side_margin, cabinet_h - (2.0 * wall_t))),
        origin=Origin(
            xyz=(
                (cabinet_w / 2.0) - (wall_t / 2.0),
                (service_open_d / 2.0) + (side_margin / 2.0),
                cabinet_h / 2.0,
            )
        ),
        material=cabinet_red,
        name="right_side_front_stile",
    )
    body.visual(
        Box((wall_t, side_margin, cabinet_h - (2.0 * wall_t))),
        origin=Origin(
            xyz=(
                (cabinet_w / 2.0) - (wall_t / 2.0),
                -((service_open_d / 2.0) + (side_margin / 2.0)),
                cabinet_h / 2.0,
            )
        ),
        material=cabinet_red,
        name="right_side_rear_stile",
    )
    body.visual(
        Box((wall_t, service_open_d, service_bottom_z - wall_t)),
        origin=Origin(
            xyz=(
                (cabinet_w / 2.0) - (wall_t / 2.0),
                0.0,
                wall_t + ((service_bottom_z - wall_t) / 2.0),
            )
        ),
        material=cabinet_red,
        name="right_side_bottom_rail",
    )
    body.visual(
        Box((wall_t, service_open_d, service_top_h)),
        origin=Origin(
            xyz=(
                (cabinet_w / 2.0) - (wall_t / 2.0),
                0.0,
                service_bottom_z + service_open_h + (service_top_h / 2.0),
            )
        ),
        material=cabinet_red,
        name="right_side_top_rail",
    )

    body.visual(
        Box((cabinet_w - (2.0 * wall_t), wall_t, front_lower_h)),
        origin=Origin(
            xyz=(0.0, front_y - (wall_t / 2.0), wall_t + (front_lower_h / 2.0))
        ),
        material=cabinet_red,
        name="front_lower_panel",
    )
    body.visual(
        Box((cabinet_w - (2.0 * wall_t), wall_t, front_header_h)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - (wall_t / 2.0),
                (cabinet_h - wall_t) - (front_header_h / 2.0),
            )
        ),
        material=cabinet_red,
        name="front_header",
    )

    front_stile_w = ((cabinet_w - (2.0 * wall_t)) - front_clear_w) / 2.0
    body.visual(
        Box((front_stile_w, wall_t, front_clear_h)),
        origin=Origin(
            xyz=(
                -((front_clear_w / 2.0) + (front_stile_w / 2.0)),
                front_y - (wall_t / 2.0),
                wall_t + front_lower_h + (front_clear_h / 2.0),
            )
        ),
        material=cabinet_red,
        name="front_left_window_stile",
    )
    body.visual(
        Box((front_stile_w, wall_t, front_clear_h)),
        origin=Origin(
            xyz=(
                (front_clear_w / 2.0) + (front_stile_w / 2.0),
                front_y - (wall_t / 2.0),
                wall_t + front_lower_h + (front_clear_h / 2.0),
            )
        ),
        material=cabinet_red,
        name="front_right_window_stile",
    )
    body.visual(
        Box((front_clear_w, 0.006, front_clear_h)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - 0.011,
                wall_t + front_lower_h + (front_clear_h / 2.0),
            )
        ),
        material=smoked_glass,
        name="front_clear_panel",
    )

    body.visual(
        Box((chute_w, chute_out, chute_floor_t)),
        origin=Origin(
            xyz=(0.0, front_y + (chute_out / 2.0), chute_bottom_z + (chute_floor_t / 2.0))
        ),
        material=zinc_gray,
        name="dispense_chute_floor",
    )
    body.visual(
        Box((chute_w, chute_out, chute_roof_t)),
        origin=Origin(
            xyz=(
                0.0,
                front_y + (chute_out / 2.0),
                chute_bottom_z + chute_open_h + (chute_roof_t / 2.0),
            )
        ),
        material=zinc_gray,
        name="dispense_chute_roof",
    )
    body.visual(
        Box((chute_side_t, chute_out, dispense_flap_h + chute_roof_t)),
        origin=Origin(
            xyz=(
                -((chute_w / 2.0) - (chute_side_t / 2.0)),
                front_y + (chute_out / 2.0),
                chute_bottom_z + ((chute_open_h + chute_roof_t) / 2.0),
            )
        ),
        material=zinc_gray,
        name="dispense_left_cheek",
    )
    body.visual(
        Box((chute_side_t, chute_out, dispense_flap_h + chute_roof_t)),
        origin=Origin(
            xyz=(
                (chute_w / 2.0) - (chute_side_t / 2.0),
                front_y + (chute_out / 2.0),
                chute_bottom_z + ((chute_open_h + chute_roof_t) / 2.0),
            )
        ),
        material=zinc_gray,
        name="dispense_right_cheek",
    )
    body.visual(
        Cylinder(radius=0.016, length=bushing_len),
        origin=Origin(
            xyz=(knob_x, front_y + (bushing_len / 2.0), knob_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=zinc_gray,
        name="knob_bushing",
    )
    body.inertial = Inertial.from_geometry(
        Box((cabinet_w, cabinet_d + chute_out, cabinet_h)),
        mass=18.0,
        origin=Origin(xyz=(0.0, chute_out / 2.0, cabinet_h / 2.0)),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.016, service_open_d, service_open_h)),
        origin=Origin(xyz=(-0.008, -(service_open_d / 2.0), service_open_h / 2.0)),
        material=cabinet_red,
        name="service_door_panel",
    )
    service_door.visual(
        Box((0.018, 0.034, 0.10)),
        origin=Origin(xyz=(0.009, -(service_open_d * 0.62), service_open_h * 0.55)),
        material=powder_black,
        name="service_door_handle",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.016, service_open_d, service_open_h)),
        mass=1.6,
        origin=Origin(xyz=(-0.008, -(service_open_d / 2.0), service_open_h / 2.0)),
    )

    dispense_flap = model.part("dispense_flap")
    dispense_flap.visual(
        Box((dispense_open_w, dispense_flap_t, dispense_flap_h)),
        origin=Origin(xyz=(0.0, -(dispense_flap_t / 2.0), dispense_flap_h / 2.0)),
        material=powder_black,
        name="dispense_flap_panel",
    )
    dispense_flap.visual(
        Box((0.070, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.009, dispense_flap_h * 0.72)),
        material=powder_black,
        name="dispense_pull_lip",
    )
    dispense_flap.inertial = Inertial.from_geometry(
        Box((dispense_open_w, 0.018, dispense_flap_h)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.001, dispense_flap_h / 2.0)),
    )

    selection_knob = model.part("selection_knob")
    selection_knob.visual(
        Cylinder(radius=0.037, length=0.026),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_cream,
        name="knob_body",
    )
    selection_knob.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_cream,
        name="knob_cap",
    )
    selection_knob.visual(
        Box((0.010, 0.026, 0.044)),
        origin=Origin(xyz=(0.022, 0.013, 0.0)),
        material=powder_black,
        name="knob_ridge",
    )
    selection_knob.inertial = Inertial.from_geometry(
        Box((0.074, 0.034, 0.074)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
    )

    model.articulation(
        "service_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(
            xyz=((cabinet_w / 2.0), service_open_d / 2.0, service_bottom_z)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "dispense_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dispense_flap,
        origin=Origin(xyz=(0.0, front_y + chute_out, chute_bottom_z + chute_floor_t)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "selection_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selection_knob,
        origin=Origin(xyz=(knob_x, front_y + bushing_len, knob_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("cabinet_body")
    service_door = object_model.get_part("service_door")
    dispense_flap = object_model.get_part("dispense_flap")
    selection_knob = object_model.get_part("selection_knob")

    service_hinge = object_model.get_articulation("service_door_hinge")
    flap_hinge = object_model.get_articulation("dispense_flap_hinge")
    knob_spin = object_model.get_articulation("selection_knob_spin")

    ctx.check(
        "service door uses vertical revolute hinge",
        service_hinge.articulation_type == ArticulationType.REVOLUTE
        and service_hinge.axis == (0.0, 0.0, 1.0)
        and service_hinge.motion_limits is not None
        and service_hinge.motion_limits.lower == 0.0
        and service_hinge.motion_limits.upper is not None
        and service_hinge.motion_limits.upper > 1.4,
        details=f"joint={service_hinge}",
    )
    ctx.check(
        "dispense flap uses horizontal revolute hinge",
        flap_hinge.articulation_type == ArticulationType.REVOLUTE
        and flap_hinge.axis == (-1.0, 0.0, 0.0)
        and flap_hinge.motion_limits is not None
        and flap_hinge.motion_limits.lower == 0.0
        and flap_hinge.motion_limits.upper is not None
        and 1.0 < flap_hinge.motion_limits.upper < 1.5,
        details=f"joint={flap_hinge}",
    )
    ctx.check(
        "selection knob uses continuous shaft rotation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.axis == (0.0, 1.0, 0.0)
        and knob_spin.motion_limits is not None
        and knob_spin.motion_limits.lower is None
        and knob_spin.motion_limits.upper is None,
        details=f"joint={knob_spin}",
    )

    ctx.expect_contact(
        selection_knob,
        body,
        elem_a="knob_body",
        elem_b="knob_bushing",
        contact_tol=0.001,
        name="selection knob seats on front bushing",
    )
    ctx.expect_contact(
        service_door,
        body,
        elem_a="service_door_panel",
        elem_b="right_side_bottom_rail",
        contact_tol=0.001,
        name="service door closes onto side opening frame",
    )

    closed_door_aabb = ctx.part_world_aabb(service_door)
    with ctx.pose({service_hinge: service_hinge.motion_limits.upper}):
        open_door_aabb = ctx.part_world_aabb(service_door)
    ctx.check(
        "service door swings outward from cabinet side",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_flap_aabb = ctx.part_world_aabb(dispense_flap)
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        open_flap_aabb = ctx.part_world_aabb(dispense_flap)
    ctx.check(
        "dispense flap drops outward from shelf opening",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > closed_flap_aabb[1][1] + 0.04
        and open_flap_aabb[1][2] < closed_flap_aabb[1][2] - 0.03,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    closed_ridge_center = _aabb_center(
        ctx.part_element_world_aabb(selection_knob, elem="knob_ridge")
    )
    with ctx.pose({knob_spin: math.pi / 2.0}):
        turned_ridge_center = _aabb_center(
            ctx.part_element_world_aabb(selection_knob, elem="knob_ridge")
        )
    ctx.check(
        "selection knob visibly rotates around its shaft",
        closed_ridge_center is not None
        and turned_ridge_center is not None
        and turned_ridge_center[2] < closed_ridge_center[2] - 0.015
        and abs(turned_ridge_center[0] - knob_spin.origin.xyz[0])
        < abs(closed_ridge_center[0] - knob_spin.origin.xyz[0]),
        details=f"closed={closed_ridge_center}, turned={turned_ridge_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
