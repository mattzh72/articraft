from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


GALVANIZED = Material("galvanized_zinc", rgba=(0.58, 0.62, 0.60, 1.0))
DARK_GASKET = Material("black_epdm", rgba=(0.02, 0.022, 0.02, 1.0))
PIN_STEEL = Material("stainless_pin", rgba=(0.82, 0.84, 0.82, 1.0))


def _box(part, name: str, size, xyz, material=GALVANIZED, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl_y(part, name: str, radius: float, length: float, xyz, material=GALVANIZED) -> None:
    """Cylinder with its long axis on local/world Y."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_roof_vent_tower")

    tower = model.part("tower")

    # One-piece stamped/molded base and tower shell: broad roof flange, low curb,
    # framed vent openings, integral louver slats, fixed hinge leaf, stops, and
    # raised bolt/clamp pads.  The intentionally simple box sections are the
    # manufacturable sheet/plastic geometry, not placeholder solids.
    _box(tower, "roof_flange", (0.80, 0.62, 0.025), (0.0, 0.0, 0.0125))
    _box(tower, "front_curb", (0.050, 0.46, 0.080), (-0.205, 0.0, 0.062))
    _box(tower, "rear_curb", (0.050, 0.46, 0.080), (0.205, 0.0, 0.062))
    _box(tower, "curb_side_0", (0.42, 0.045, 0.080), (0.0, -0.230, 0.062))
    _box(tower, "curb_side_1", (0.42, 0.045, 0.080), (0.0, 0.230, 0.062))

    _box(tower, "rear_panel", (0.045, 0.47, 0.46), (0.205, 0.0, 0.320))
    _box(tower, "front_jamb_0", (0.050, 0.060, 0.36), (-0.205, -0.205, 0.270))
    _box(tower, "front_jamb_1", (0.050, 0.060, 0.36), (-0.205, 0.205, 0.270))
    _box(tower, "front_header", (0.055, 0.47, 0.065), (-0.205, 0.0, 0.472))
    _box(tower, "front_sill", (0.055, 0.47, 0.055), (-0.205, 0.0, 0.118))

    for side, y in (("0", -0.238), ("1", 0.238)):
        _box(tower, f"side_lower_rail_{side}", (0.39, 0.040, 0.060), (0.005, y, 0.120))
        _box(tower, f"side_upper_rail_{side}", (0.39, 0.040, 0.080), (0.005, y, 0.505))
        _box(tower, f"side_front_post_{side}", (0.060, 0.040, 0.46), (-0.175, y, 0.320))
        _box(tower, f"side_rear_post_{side}", (0.060, 0.040, 0.46), (0.185, y, 0.320))
        for idx, z in enumerate((0.220, 0.305, 0.390)):
            _box(
                tower,
                f"side_louver_{side}_{idx}",
                (0.34, 0.020, 0.024),
                (0.005, y, z),
                rpy=(0.36 if y < 0 else -0.36, 0.0, 0.0),
            )

    _box(tower, "overhanging_cap", (0.47, 0.53, 0.040), (0.005, 0.0, 0.570))
    _box(tower, "front_drip_edge", (0.055, 0.51, 0.026), (-0.246, 0.0, 0.537))

    # Raised pads represent the low-part-count roof clamp/bolt interface.
    for i, (x, y) in enumerate(((-0.315, -0.235), (-0.315, 0.235), (0.315, -0.235), (0.315, 0.235))):
        _box(tower, f"bolt_pad_{i}", (0.090, 0.070, 0.010), (x, y, 0.030))
        _cyl_y(tower, f"bolt_head_{i}", 0.018, 0.010, (x, y, 0.043), material=PIN_STEEL)

    # Fixed hinge leaf, alternating fixed barrels, through pin, and explicit
    # close/open stop features all remain integral to the tower part.
    for i, (y, length) in enumerate(((-0.165, 0.070), (0.000, 0.080), (0.165, 0.070))):
        _box(tower, f"fixed_hinge_plate_{i}", (0.040, length, 0.008), (-0.232, y, 0.445))
    for name, y, length in (
        ("fixed_barrel_0", -0.165, 0.070),
        ("fixed_barrel_1", 0.000, 0.080),
        ("fixed_barrel_2", 0.165, 0.070),
    ):
        _cyl_y(tower, name, 0.015, length, (-0.245, y, 0.455), material=GALVANIZED)
    _cyl_y(tower, "hinge_pin", 0.0045, 0.440, (-0.245, 0.0, 0.455), material=PIN_STEEL)

    for i, y in enumerate((-0.120, 0.120)):
        _box(tower, f"closed_stop_{i}", (0.050, 0.040, 0.034), (-0.233, y, 0.1625), material=DARK_GASKET)
        _box(tower, f"open_stop_{i}", (0.070, 0.030, 0.024), (-0.235, 0.205 if y > 0 else -0.205, 0.515), rpy=(0.0, -0.50, 0.0))
        _box(tower, f"snap_lug_{i}", (0.024, 0.040, 0.018), (-0.238, y, 0.090))

    flap = model.part("flap")

    # Single stamped weather flap with turned side returns, drip lip, shallow
    # stiffening ribs, and the moving hinge leaf/barrels.  Its part frame is the
    # hinge line, so the panel naturally swings from the explicit barrels.
    _box(flap, "flap_panel", (0.014, 0.360, 0.325), (-0.021, 0.0, -0.1775))
    _box(flap, "side_return_0", (0.040, 0.012, 0.315), (-0.009, -0.185, -0.180))
    _box(flap, "side_return_1", (0.040, 0.012, 0.315), (-0.009, 0.185, -0.180))
    _box(flap, "bottom_drip_lip", (0.052, 0.372, 0.018), (-0.022, 0.0, -0.342))
    for i, y in enumerate((-0.085, 0.085)):
        _box(flap, f"moving_hinge_plate_{i}", (0.040, 0.070, 0.008), (-0.020, y, -0.012))
    for i, y in enumerate((-0.085, 0.085)):
        _cyl_y(flap, f"moving_barrel_{i}", 0.015, 0.075, (0.0, y, 0.0), material=GALVANIZED)
        _box(flap, f"hinge_strap_{i}", (0.045, 0.058, 0.009), (-0.020, y, -0.015))
    for i, y in enumerate((-0.095, 0.095)):
        _box(flap, f"stiffening_rib_{i}", (0.006, 0.018, 0.270), (-0.030, y, -0.190))
    for i, y in enumerate((-0.193, 0.193)):
        _box(flap, f"stop_ear_{i}", (0.034, 0.010, 0.044), (-0.006, y, -0.038))

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(-0.245, 0.0, 0.455)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("flap_hinge")

    for barrel in ("moving_barrel_0", "moving_barrel_1"):
        ctx.allow_overlap(
            tower,
            flap,
            elem_a="hinge_pin",
            elem_b=barrel,
            reason="The stainless hinge pin is intentionally captured through the flap barrel bore.",
        )
        ctx.expect_within(
            tower,
            flap,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem=barrel,
            margin=0.001,
            name=f"pin centered in {barrel}",
        )
        ctx.expect_overlap(
            tower,
            flap,
            axes="y",
            elem_a="hinge_pin",
            elem_b=barrel,
            min_overlap=0.070,
            name=f"pin spans {barrel}",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            tower,
            flap,
            axis="x",
            positive_elem="closed_stop_0",
            negative_elem="flap_panel",
            min_gap=0.0003,
            max_gap=0.004,
            name="closed flap rests near bumper 0",
        )
        ctx.expect_gap(
            tower,
            flap,
            axis="x",
            positive_elem="closed_stop_1",
            negative_elem="flap_panel",
            min_gap=0.0003,
            max_gap=0.004,
            name="closed flap rests near bumper 1",
        )
        panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
        opening_ok = False
        if panel_aabb is not None:
            mn, mx = panel_aabb
            opening_ok = (
                mn[1] < -0.168
                and mx[1] > 0.168
                and mn[2] < 0.140
                and mx[2] > 0.425
            )
        ctx.check("closed flap covers framed opening", opening_ok, details=f"panel_aabb={panel_aabb}")

    rest_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
        opens_ok = False
        if rest_aabb is not None and open_aabb is not None:
            rest_mn, rest_mx = rest_aabb
            open_mn, open_mx = open_aabb
            rest_center_x = 0.5 * (rest_mn[0] + rest_mx[0])
            open_center_x = 0.5 * (open_mn[0] + open_mx[0])
            opens_ok = open_mn[2] > rest_mn[2] + 0.180 and open_center_x < rest_center_x - 0.035
        ctx.check("flap opens upward and outward", opens_ok, details=f"rest={rest_aabb}, open={open_aabb}")

    return ctx.report()


object_model = build_object_model()
